// ---------------------------------------------------------------------------------------------
// LisaNet SD server -- ESProFile's personality in ESP32 MODE = WIFI HDD. NOT part of upstream ESProFile; written with
// an AI assistant. Ported from the same file in the LisaFPGA fork's ESFloppy (wottle/ESFloppy, branch lisanet).
// Implements docs/lisanet_link_protocol.md (LisaFPGA fork) on two transports at once: the link UART to
// the external WiFi board (through the FPGA) and the USB CDC port (for development from a PC). No ProFile emulation
// runs in this personality, so nothing else touches the SD card (and the Lisa has no hard disk meanwhile).
// ---------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include "lisanet.h"
#include "PinDefs_LisaFPGA.h"   // SD card pins

#define SDS_VERSION      "0.1"
#define PROTO_VERSION    1
#define MAX_DATA         4096
#define MAX_PAYLOAD      (MAX_DATA + 64)

enum : uint8_t { ST_OK = 0, ST_NOT_FOUND, ST_EXISTS, ST_IO, ST_BAD_REQUEST, ST_NO_HANDLE, ST_UNKNOWN_CMD, ST_NO_CARD,
                 ST_NO_CONTIGUOUS };
enum : uint8_t {
    CMD_HELLO = 0x01, CMD_INFO = 0x02, CMD_LIST = 0x10, CMD_STAT = 0x11, CMD_OPEN = 0x20, CMD_READ = 0x21,
    CMD_WRITE = 0x22, CMD_CLOSE = 0x23, CMD_DELETE = 0x30, CMD_RENAME = 0x31, CMD_MKDIR = 0x32, CMD_COPY_START = 0x33, CMD_COPY_STEP = 0x34,
    CMD_MOUNT_NEXT_SET = 0x40, CMD_MOUNT_NEXT_GET = 0x41
};

// ESProFile's SD objects (ESProFile_Emulator.ino); the emulator never runs in this personality, so they are ours alone
extern SPIClass SD_SPI;
extern SdFat32 SDCard;

static HardwareSerial Link(1);
static bool cardOk = false;
static File32 openFile;
static bool fileOpen = false;
static const uint8_t HANDLE = 1;

// ---- CRC-32 (IEEE, zlib-compatible) ------------------------------------------------------------
static uint32_t crcTable[256];
static void crcInit() {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crcTable[i] = c;
    }
}
static uint32_t crc32(const uint8_t *p, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    while (n--) c = crcTable[(c ^ *p++) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

// ---- Little-endian helpers ---------------------------------------------------------------------
static uint16_t rd16(const uint8_t *p) { return p[0] | (uint16_t)p[1] << 8; }
static uint64_t rd64(const uint8_t *p) { uint64_t v = 0; for (int i = 7; i >= 0; i--) v = (v << 8) | p[i]; return v; }
static void wr16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void wr64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) { p[i] = v; v >>= 8; } }

// ---- Response building -------------------------------------------------------------------------
static uint8_t resp[MAX_PAYLOAD];
static size_t respLen;
static void begin(uint8_t status) { resp[0] = status; respLen = 1; }
static void put8(uint8_t v)  { if (respLen < MAX_PAYLOAD) resp[respLen++] = v; }
static void put16(uint16_t v) { if (respLen + 2 <= MAX_PAYLOAD) { wr16(resp + respLen, v); respLen += 2; } }
static void put64(uint64_t v) { if (respLen + 8 <= MAX_PAYLOAD) { wr64(resp + respLen, v); respLen += 8; } }
static void putBytes(const void *d, size_t n) { if (respLen + n <= MAX_PAYLOAD) { memcpy(resp + respLen, d, n); respLen += n; } }

static void sendFrame(Stream &s, uint8_t cmd, uint8_t seq) {
    uint8_t hdr[6] = {0xA5, 0x5A, cmd, seq, 0, 0};
    wr16(hdr + 4, respLen);
    // CRC over cmd, seq, len and payload
    uint32_t c = 0xFFFFFFFFu;
    for (int i = 2; i < 6; i++) c = crcTable[(c ^ hdr[i]) & 0xFF] ^ (c >> 8);
    for (size_t i = 0; i < respLen; i++) c = crcTable[(c ^ resp[i]) & 0xFF] ^ (c >> 8);
    c ^= 0xFFFFFFFFu;
    uint8_t tail[4] = {(uint8_t)c, (uint8_t)(c >> 8), (uint8_t)(c >> 16), (uint8_t)(c >> 24)};
    s.write(hdr, 6);
    s.write(resp, respLen);
    s.write(tail, 4);
}

// Copies a path out of a payload into a NUL-terminated buffer; empty path means "/"
static bool getPath(const uint8_t *p, size_t n, char *out, size_t cap) {
    if (n >= cap) return false;
    memcpy(out, p, n);
    out[n] = 0;
    if (n == 0) strcpy(out, "/");
    return true;
}

// Staging. An upload (OPEN mode 3) or a copy is written under a temporary name in the hidden /.LISANET folder and moved
// onto its real name only once complete, inside ONE request (the CLOSE, or the last COPY_STEP). The server checks MODE
// between requests, so a mode change -- e.g. the Lisa being switched on in WIFI HDD -- can only land before or after
// that move: the target is always either the old file or the complete new one, never a half-written image. Temporary
// files left by an interruption are deleted at the next start.
#define STAGE_DIR   "/.LISANET"
#define UPLOAD_TEMP STAGE_DIR "/UPLOAD.PRT"
#define COPY_TEMP   STAGE_DIR "/COPY.PRT"
static bool uploadStaged = false;
static char uploadTarget[256];

// Moves a finished staged file onto target. An existing target is first renamed to backup, or deleted if backup is
// empty or null.
static uint8_t commitStaged(const char *temp, const char *target, const char *backup) {
    if (SDCard.exists(target)) {
        File32 t;
        if (t.open(target, O_RDONLY)) { bool dir = t.isDir(); t.close(); if (dir) return ST_EXISTS; }
        if (backup && *backup) {
            if (SDCard.exists(backup)) return ST_EXISTS;
            if (!SDCard.rename(target, backup)) return ST_IO;
        } else if (!SDCard.remove(target)) {
            return ST_IO;
        }
    }
    return SDCard.rename(temp, target) ? ST_OK : ST_IO;
}

// Closes whatever is open WITHOUT completing it: a staged upload is discarded (its target is untouched); a plain file
// is synced and closed.
static void closeOpenFile() {
    if (!fileOpen) return;
    if (uploadStaged) { openFile.close(); SDCard.remove(UPLOAD_TEMP); uploadStaged = false; }
    else { openFile.sync(); openFile.close(); }
    fileOpen = false;
}

// On-card copy (COPY_START / COPY_STEP): one copy at a time, done in pieces so each request answers quickly and the
// client can show progress. Staged like an upload (see above) and reserved contiguously first; a copy that fails or is
// abandoned (MODE dropping, or a new COPY_START) just deletes its temporary file.
#define COPY_STEP_BYTES (1024UL * 1024UL)       // per COPY_STEP: about a second of card time
static File32 copySrc, copyDst;
static bool copying = false;
static uint64_t copyDone = 0, copyTotal = 0;
static char copyDstPath[256];
static bool copyReplace = false;                // the target may already exist (it is then replaced)
static char copyBackup[256];                    // ...and renamed to this first, or deleted if empty
static uint8_t copyBuf[16384];

static void copyAbort() {
    if (!copying) return;
    copySrc.close();
    copyDst.close();
    SDCard.remove(COPY_TEMP);
    copying = false;
}

// ---- Command handlers --------------------------------------------------------------------------
static void handle(uint8_t cmd, const uint8_t *p, size_t n) {
    char path[256], path2[256];
    if (!cardOk && cmd != CMD_HELLO) { begin(ST_NO_CARD); return; }
    switch (cmd) {
    case CMD_HELLO: {
        begin(ST_OK); put8(PROTO_VERSION); put16(MAX_DATA);
        const char *fw = "ESProFile + LisaNet SD server " SDS_VERSION;
        putBytes(fw, strlen(fw));
        break;
    }
    case CMD_INFO: {
        uint64_t total = (uint64_t)SDCard.card()->sectorCount() * 512;
        int32_t freeClusters = SDCard.freeClusterCount();
        uint64_t freeBytes = freeClusters < 0 ? 0 : (uint64_t)freeClusters * SDCard.bytesPerCluster();
        begin(ST_OK); put64(total); put64(freeBytes);
        break;
    }
    case CMD_LIST: {
        if (n < 2 || !getPath(p + 2, n - 2, path, sizeof path)) { begin(ST_BAD_REQUEST); return; }
        uint16_t start = rd16(p);
        File32 dir;
        if (!dir.open(path, O_RDONLY) || !dir.isDir()) { begin(ST_NOT_FOUND); return; }
        begin(ST_OK);
        size_t nextPos = respLen;
        put16(0xFFFF);                                   // patched below if the page fills up
        File32 e;
        uint16_t idx = 0;
        while (e.openNext(&dir, O_RDONLY)) {
            if (idx++ < start) { e.close(); continue; }
            char name[128];
            size_t nl = e.getName(name, sizeof name);
            if (respLen + 1 + 8 + 1 + nl > MAX_PAYLOAD - 8) { wr16(resp + nextPos, idx - 1); e.close(); break; }
            put8(e.isDir() ? 1 : 0); put64(e.fileSize()); put8(nl); putBytes(name, nl);
            e.close();
        }
        dir.close();
        break;
    }
    case CMD_STAT: {
        if (!getPath(p, n, path, sizeof path)) { begin(ST_BAD_REQUEST); return; }
        File32 f;
        if (!f.open(path, O_RDONLY)) { begin(ST_NOT_FOUND); return; }
        begin(ST_OK); put8(f.isDir() ? 1 : 0); put64(f.fileSize());
        f.close();
        break;
    }
    case CMD_OPEN: {
        // Modes: 0 read, 1 create + truncate, 2 create keeping contents, 3 create + truncate + reserve u64 size bytes
        // contiguously. ESProFile reads and writes images through the file system, so it does not need this, but the
        // WiFi board always uploads with mode 3 (ESFloppy does need it) and a contiguous image is no worse here.
        if (n < 1) { begin(ST_BAD_REQUEST); return; }
        uint8_t mode = p[0];
        size_t pathAt = (mode == 3) ? 9 : 1;
        if (mode > 3 || n < pathAt || !getPath(p + pathAt, n - pathAt, path, sizeof path)) { begin(ST_BAD_REQUEST); return; }
        closeOpenFile();
        oflag_t fl = mode == 0 ? O_RDONLY : mode == 2 ? (O_RDWR | O_CREAT) : (O_RDWR | O_CREAT | O_TRUNC);
        const char *openPath = path;
        if (mode == 3) {                         // staged: written under a temporary name, moved into place by CLOSE
            SDCard.mkdir(STAGE_DIR, true);
            SDCard.remove(UPLOAD_TEMP);
            strcpy(uploadTarget, path);
            openPath = UPLOAD_TEMP;
        }
        if (!openFile.open(openPath, fl)) { begin(mode == 0 ? ST_NOT_FOUND : ST_IO); return; }
        if (mode == 3) {
            uint64_t size = rd64(p + 1);
            if (size && !openFile.preAllocate(size)) {
                openFile.close();
                SDCard.remove(UPLOAD_TEMP);
                begin(ST_NO_CONTIGUOUS);
                return;
            }
            uploadStaged = true;
        }
        fileOpen = true;
        begin(ST_OK); put8(HANDLE); put64(openFile.fileSize());
        break;
    }
    case CMD_READ: {
        if (n != 11) { begin(ST_BAD_REQUEST); return; }
        if (!fileOpen || p[0] != HANDLE) { begin(ST_NO_HANDLE); return; }
        uint64_t off = rd64(p + 1);
        uint16_t cnt = rd16(p + 9);
        if (cnt > MAX_DATA) cnt = MAX_DATA;
        if (!openFile.seekSet(off)) { begin(ST_IO); return; }
        begin(ST_OK);
        int got = openFile.read(resp + respLen, cnt);
        if (got < 0) { begin(ST_IO); return; }
        respLen += got;
        break;
    }
    case CMD_WRITE: {
        if (n < 9 || n - 9 > MAX_DATA) { begin(ST_BAD_REQUEST); return; }
        if (!fileOpen || p[0] != HANDLE) { begin(ST_NO_HANDLE); return; }
        uint64_t off = rd64(p + 1);
        if (!openFile.seekSet(off)) { begin(ST_IO); return; }
        size_t w = openFile.write(p + 9, n - 9);
        if (w != n - 9) { begin(ST_IO); return; }
        begin(ST_OK); put16(w);
        break;
    }
    case CMD_CLOSE: {
        // u8 handle [, u8 abandon]. Completes the open file; for a staged upload that means moving it into place.
        // abandon = 1 instead discards a staged upload (the client gave up on it), leaving its target untouched.
        if (!fileOpen) { begin(ST_OK); return; }       // repeating a CLOSE is harmless
        if (n >= 2 && p[1] == 1) { closeOpenFile(); begin(ST_OK); return; }
        bool ok = openFile.sync();
        openFile.close();
        fileOpen = false;
        if (uploadStaged) {
            uploadStaged = false;
            if (!ok) { SDCard.remove(UPLOAD_TEMP); begin(ST_IO); return; }
            begin(commitStaged(UPLOAD_TEMP, uploadTarget, nullptr));
            return;
        }
        begin(ok ? ST_OK : ST_IO);
        break;
    }
    case CMD_DELETE: {
        if (!getPath(p, n, path, sizeof path)) { begin(ST_BAD_REQUEST); return; }
        File32 f;
        if (!f.open(path, O_RDONLY)) { begin(ST_NOT_FOUND); return; }
        bool isDir = f.isDir();
        f.close();
        begin((isDir ? SDCard.rmdir(path) : SDCard.remove(path)) ? ST_OK : ST_IO);
        break;
    }
    case CMD_RENAME: {
        const uint8_t *z = (const uint8_t *)memchr(p, 0, n);
        if (!z || !getPath(p, z - p, path, sizeof path) || !getPath(z + 1, n - (z - p) - 1, path2, sizeof path2)) {
            begin(ST_BAD_REQUEST); return;
        }
        if (!SDCard.exists(path)) { begin(ST_NOT_FOUND); return; }
        if (SDCard.exists(path2)) { begin(ST_EXISTS); return; }
        begin(SDCard.rename(path, path2) ? ST_OK : ST_IO);
        break;
    }
    case CMD_MKDIR: {
        if (!getPath(p, n, path, sizeof path)) { begin(ST_BAD_REQUEST); return; }
        if (SDCard.exists(path)) { begin(ST_EXISTS); return; }
        begin(SDCard.mkdir(path, true) ? ST_OK : ST_IO);
        break;
    }
    // Mount-next is ESFloppy's: ESProFile always starts on profile.image and the Selector picks the rest. GET answers
    // "none" so a client can ask either server; SET is refused.
    case CMD_MOUNT_NEXT_GET:
        begin(ST_OK);
        break;
    case CMD_COPY_START: {
        // source, NUL, target [, NUL, backup]. With the third field the target may already exist: once the copy is
        // complete it is renamed to backup (or deleted, if backup is empty) and replaced, in the same request.
        const uint8_t *z = (const uint8_t *)memchr(p, 0, n);
        if (!z) { begin(ST_BAD_REQUEST); return; }
        const uint8_t *d = z + 1;
        size_t dn = n - (d - p);
        const uint8_t *z2 = (const uint8_t *)memchr(d, 0, dn);
        bool replace = z2 != nullptr;
        size_t dlen = replace ? (size_t)(z2 - d) : dn;
        size_t blen = replace ? dn - dlen - 1 : 0;
        if (!getPath(p, z - p, path, sizeof path) || !getPath(d, dlen, path2, sizeof path2) || blen >= sizeof copyBackup) {
            begin(ST_BAD_REQUEST); return;
        }
        copyAbort();
        if (replace) memcpy(copyBackup, z2 + 1, blen);
        copyBackup[blen] = 0;
        copyReplace = replace;
        if (!SDCard.exists(path)) { begin(ST_NOT_FOUND); return; }
        if (!replace && SDCard.exists(path2)) { begin(ST_EXISTS); return; }
        if (copyBackup[0] && SDCard.exists(copyBackup)) { begin(ST_EXISTS); return; }
        if (!copySrc.open(path, O_RDONLY)) { begin(ST_IO); return; }
        if (copySrc.isDir()) { copySrc.close(); begin(ST_BAD_REQUEST); return; }
        SDCard.mkdir(STAGE_DIR, true);
        SDCard.remove(COPY_TEMP);
        if (!copyDst.open(COPY_TEMP, O_RDWR | O_CREAT | O_TRUNC)) { copySrc.close(); begin(ST_IO); return; }
        strcpy(copyDstPath, path2);
        copyTotal = copySrc.fileSize();
        copyDone = 0;
        copying = true;
        if (copyTotal && !copyDst.preAllocate(copyTotal)) { copyAbort(); begin(ST_NO_CONTIGUOUS); return; }
        begin(ST_OK); put64(copyTotal);
        break;
    }
    case CMD_COPY_STEP: {
        if (!copying) { begin(ST_NO_HANDLE); return; }
        uint64_t stepEnd = copyDone + COPY_STEP_BYTES;
        while (copyDone < copyTotal && copyDone < stepEnd) {
            size_t want = (size_t)min((uint64_t)sizeof copyBuf, copyTotal - copyDone);
            int got = copySrc.read(copyBuf, want);
            if (got <= 0 || copyDst.write(copyBuf, got) != (size_t)got) { copyAbort(); begin(ST_IO); return; }
            copyDone += got;
        }
        if (copyDone == copyTotal) {
            bool ok = copyDst.sync();
            copySrc.close();
            copyDst.close();
            copying = false;
            if (!ok) { SDCard.remove(COPY_TEMP); begin(ST_IO); return; }
            if (!copyReplace && SDCard.exists(copyDstPath)) { SDCard.remove(COPY_TEMP); begin(ST_EXISTS); return; }
            uint8_t st = commitStaged(COPY_TEMP, copyDstPath, copyBackup);   // the move into place
            if (st != ST_OK) { SDCard.remove(COPY_TEMP); begin(st); return; }
        }
        begin(ST_OK); put64(copyDone); put64(copyTotal);
        break;
    }
    default:
        begin(ST_UNKNOWN_CMD);
    }
}

// ---- Frame receiver, one per transport -----------------------------------------------------------
struct Receiver {
    Stream *s;
    uint8_t buf[6 + MAX_PAYLOAD + 4];
    size_t have = 0;
    size_t need = 0;   // total frame length once the header is in
    uint32_t lastByte = 0;

    void poll() {
        // A frame that stops arriving part-way is abandoned after 100 ms of silence. Without this, a damaged large frame
        // leaves the receiver waiting for bytes that never come, and it swallows the next requests as its missing data.
        if (have && millis() - lastByte > 100) have = 0;
        while (s->available()) {
            uint8_t b = s->read();
            lastByte = millis();
            if (have == 0 && b != 0xA5) continue;                  // hunt for sync; skips anything else
            if (have == 1 && b != 0x5A) { have = (b == 0xA5) ? 1 : 0; continue; }
            buf[have++] = b;
            if (have == 6) {
                uint16_t len = rd16(buf + 4);
                if (len > MAX_PAYLOAD) { have = 0; continue; }     // impossible length: drop and resync
                need = 6 + len + 4;
            }
            if (have >= 6 && have == need) {
                frame();
                have = 0;
            }
        }
    }

    void frame() {
        uint16_t len = rd16(buf + 4);
        uint32_t got = buf[6 + len] | (uint32_t)buf[7 + len] << 8 | (uint32_t)buf[8 + len] << 16 | (uint32_t)buf[9 + len] << 24;
        if (crc32(buf + 2, 4 + len) != got) return;                // corrupt: no answer, the client resends
        uint8_t cmd = buf[2], seq = buf[3];
        if (cmd & 0x80) return;                                    // a response, not a request
        handle(cmd, buf + 6, len);
        sendFrame(*s, cmd | 0x80, seq);
    }
};

static Receiver usbRx, linkRx;

[[noreturn]] void sdServerMain() {
    crcInit();
#ifdef red_led
    // ESProFile: amber (red + green) while serving, so the LED tells the personalities apart (green = the hard disk)
    analogWrite(red_led, red_pwm_duty);
    analogWrite(green_led, green_pwm_duty);
#endif
    // USB: a 4 KB READ response must go out whole. ESProFile's console sets a zero TX timeout (never block), which
    // silently drops whatever does not fit the small default buffer -- so give it a big buffer and let it wait.
    Serial.setTxBufferSize(8192);
    Serial.setRxBufferSize(8192);                                   // and a 4 KB WRITE request must fit on the way in
    Serial.begin(115200);
    Serial.setTxTimeoutMs(500);
    Link.setRxBufferSize(8192);
    Link.setTxBufferSize(8192);
    Link.begin(LISANET_LINK_BAUD, SERIAL_8N1, LISANET_LINK_RX_PIN, LISANET_LINK_TX_PIN);
    Serial.println("\r\n[sdServer] LisaNet SD server " SDS_VERSION " (ESP32 MODE = WIFI HDD)");

    SD_SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
    cardOk = SDCard.begin(SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(20), &SD_SPI));
    Serial.println(cardOk ? "[sdServer] SD card mounted" : "[sdServer] SD card NOT mounted");
    if (cardOk) { SDCard.remove(UPLOAD_TEMP); SDCard.remove(COPY_TEMP); }   // left by an interrupted upload or copy

    usbRx.s = &Serial;
    linkRx.s = &Link;
    for (;;) {
        // MODE dropped (FLOPPY or WIFI FDD): close any open file (synced) and restart into ESProFile
        if (lisanetModeStableAt(LOW)) {
            closeOpenFile();
            copyAbort();
            Serial.println("[sdServer] MODE dropped: restarting into ESProFile");
            Serial.flush();
            delay(20);
            ESP.restart();
        }
        usbRx.poll();
        linkRx.poll();
        if (!Serial.available() && !Link.available()) delay(1);   // idle: let the other tasks run
    }
}
