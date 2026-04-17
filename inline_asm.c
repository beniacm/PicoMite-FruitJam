/*
 * Inline ARM Thumb-2 Assembler for PicoMite on RP2350
 * Supports a subset of Thumb-2 instructions for writing tight loops
 * and direct hardware register access from BASIC.
 *
 * Usage:
 *   DIM INTEGER code%(20)
 *   ASM code%()
 *     MOV R0, #42
 *     BX LR
 *   END ASM
 *   PRINT CALL(PEEK(VARADDR code%()), 0)   ' returns 42
 */

#ifdef ADAFRUIT_FRUIT_JAM

#include "MMBasic_Includes.h"
#include "Hardware_Includes.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

// Maximum labels and code size
#define ASM_MAX_LABELS 32
#define ASM_MAX_CODE   1024  // max bytes of generated code
#define ASM_MAX_FIXUPS 32

typedef struct {
    char name[32];
    int offset;
} asm_label_t;

typedef struct {
    int offset;       // offset in code where fixup is needed
    char label[32];   // target label name
    int type;         // 0 = B (unconditional), 1 = Bcc (conditional)
    uint8_t cond;     // condition code for Bcc
} asm_fixup_t;

static uint8_t *asm_code;
static int asm_pos;
static asm_label_t *asm_labels;  // allocated via GetTempMemory
static int asm_nlabels;
static asm_fixup_t *asm_fixups;  // allocated via GetTempMemory
static int asm_nfixups;
static int asm_pass;

static void asm_emit16(uint16_t instr) {
    if (asm_pass == 1 && asm_pos + 2 <= ASM_MAX_CODE) {
        asm_code[asm_pos] = instr & 0xFF;
        asm_code[asm_pos + 1] = (instr >> 8) & 0xFF;
    }
    asm_pos += 2;
}

static void asm_emit32(uint32_t instr) {
    // Thumb-2 32-bit: high halfword first, then low halfword
    asm_emit16((instr >> 16) & 0xFFFF);
    asm_emit16(instr & 0xFFFF);
}

static void asm_add_label(const char *name, int offset) {
    for (int i = 0; i < asm_nlabels; i++) {
        if (strcasecmp(asm_labels[i].name, name) == 0) {
            asm_labels[i].offset = offset;
            return;
        }
    }
    if (asm_nlabels < ASM_MAX_LABELS) {
        strncpy(asm_labels[asm_nlabels].name, name, 31);
        asm_labels[asm_nlabels].offset = offset;
        asm_nlabels++;
    }
}

static int asm_find_label(const char *name) {
    for (int i = 0; i < asm_nlabels; i++) {
        if (strcasecmp(asm_labels[i].name, name) == 0)
            return asm_labels[i].offset;
    }
    return -1;
}

// Skip whitespace
static char *skip_ws(char *p) {
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

// Parse register name: R0-R15, SP, LR, PC
static int parse_reg(char **pp) {
    char *p = skip_ws(*pp);
    int reg = -1;

    if ((*p == 'R' || *p == 'r') && isdigit(p[1])) {
        reg = atoi(p + 1);
        p += 2;
        if (reg >= 10) p++;
        if (reg > 15) return -1;
    } else if (strncasecmp(p, "SP", 2) == 0 && !isalnum(p[2])) {
        reg = 13; p += 2;
    } else if (strncasecmp(p, "LR", 2) == 0 && !isalnum(p[2])) {
        reg = 14; p += 2;
    } else if (strncasecmp(p, "PC", 2) == 0 && !isalnum(p[2])) {
        reg = 15; p += 2;
    }
    *pp = p;
    return reg;
}

// Parse immediate: #decimal or #0xhex
static int parse_imm(char **pp, int *val) {
    char *p = skip_ws(*pp);
    if (*p != '#') return 0;
    p++;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X'))
        *val = strtol(p, &p, 16);
    else if (p[0] == '-')
        *val = strtol(p, &p, 10);
    else
        *val = strtol(p, &p, 10);
    *pp = p;
    return 1;
}

// Expect and skip a comma
static int expect_comma(char **pp) {
    char *p = skip_ws(*pp);
    if (*p != ',') return 0;
    *pp = p + 1;
    return 1;
}

// Parse register list for PUSH/POP: {R0, R1, LR}
static int parse_reglist(char **pp) {
    char *p = skip_ws(*pp);
    if (*p != '{') return -1;
    p++;
    int mask = 0;
    while (1) {
        int r = parse_reg(&p);
        if (r < 0) return -1;
        mask |= (1 << r);
        p = skip_ws(p);
        if (*p == '-') {
            p++;
            int r2 = parse_reg(&p);
            if (r2 < 0 || r2 <= r) return -1;
            for (int i = r; i <= r2; i++) mask |= (1 << i);
            p = skip_ws(p);
        }
        if (*p == '}') { p++; break; }
        if (*p != ',') return -1;
        p++;
    }
    *pp = p;
    return mask;
}

// Parse a label operand (for branches)
static int parse_label_operand(char **pp, char *label_out) {
    char *p = skip_ws(*pp);
    int i = 0;
    while (isalnum(*p) || *p == '_') {
        if (i < 31) label_out[i++] = *p;
        p++;
    }
    label_out[i] = 0;
    *pp = p;
    return i > 0;
}

// Condition codes for Bcc
static int parse_condition(const char *s) {
    static const char *conds[] = {
        "EQ","NE","CS","CC","MI","PL","VS","VC",
        "HI","LS","GE","LT","GT","LE","AL",NULL
    };
    // Also accept HS=CS, LO=CC
    if (strncasecmp(s, "HS", 2) == 0) return 2;
    if (strncasecmp(s, "LO", 2) == 0) return 3;
    for (int i = 0; conds[i]; i++) {
        if (strncasecmp(s, conds[i], 2) == 0) return i;
    }
    return -1;
}

// Assemble one instruction line
static void asm_line(char *line) {
    char *p = skip_ws(line);
    if (*p == 0 || *p == ';' || *p == '\'') return; // empty or comment

    // Check for label (word followed by ':')
    char *colon = strchr(p, ':');
    if (colon && colon > p) {
        char label[32];
        int len = colon - p;
        if (len > 31) len = 31;
        memcpy(label, p, len);
        label[len] = 0;
        // Verify it's a valid label (not an instruction)
        bool is_label = true;
        for (int i = 0; i < len; i++) {
            if (!isalnum(label[i]) && label[i] != '_') { is_label = false; break; }
        }
        if (is_label) {
            asm_add_label(label, asm_pos);
            p = colon + 1;
            p = skip_ws(p);
            if (*p == 0) return;
        }
    }

    // Parse mnemonic
    char mnem[16];
    int mi = 0;
    while (isalpha(*p) && mi < 15) mnem[mi++] = toupper(*p++);
    mnem[mi] = 0;

    // ---- NOP ----
    if (strcmp(mnem, "NOP") == 0) {
        asm_emit16(0xBF00);
        return;
    }

    // ---- BX reg ----
    if (strcmp(mnem, "BX") == 0) {
        int rm = parse_reg(&p);
        if (rm < 0) error("ASM: bad register for BX");
        asm_emit16(0x4700 | (rm << 3));
        return;
    }

    // ---- BLX reg ----
    if (strcmp(mnem, "BLX") == 0) {
        int rm = parse_reg(&p);
        if (rm < 0) error("ASM: bad register for BLX");
        asm_emit16(0x4780 | (rm << 3));
        return;
    }

    // ---- PUSH {reglist} ----
    if (strcmp(mnem, "PUSH") == 0) {
        int mask = parse_reglist(&p);
        if (mask < 0) error("ASM: bad register list for PUSH");
        int lo = mask & 0xFF;
        int lr = (mask >> 14) & 1;
        asm_emit16(0xB400 | (lr << 8) | lo);
        return;
    }

    // ---- POP {reglist} ----
    if (strcmp(mnem, "POP") == 0) {
        int mask = parse_reglist(&p);
        if (mask < 0) error("ASM: bad register list for POP");
        int lo = mask & 0xFF;
        int pc = (mask >> 15) & 1;
        asm_emit16(0xBC00 | (pc << 8) | lo);
        return;
    }

    // ---- MOV Rd, #imm8 / MOV Rd, Rs ----
    if (strcmp(mnem, "MOV") == 0 || strcmp(mnem, "MOVS") == 0) {
        int rd = parse_reg(&p);
        if (rd < 0) error("ASM: bad Rd for MOV");
        expect_comma(&p);
        int imm;
        if (parse_imm(&p, &imm)) {
            if (rd <= 7 && imm >= 0 && imm <= 255) {
                asm_emit16(0x2000 | (rd << 8) | (imm & 0xFF)); // MOVS Rd, #imm8
            } else {
                // MOV.W Rd, #imm (T2 encoding: 32-bit)
                // Simplified: MOVW Rd, #imm16
                if (imm >= 0 && imm <= 65535) {
                    uint16_t imm16 = imm;
                    uint32_t i = (imm16 >> 11) & 1;
                    uint32_t imm4 = (imm16 >> 12) & 0xF;
                    uint32_t imm3 = (imm16 >> 8) & 0x7;
                    uint32_t imm8 = imm16 & 0xFF;
                    asm_emit32(0xF2400000 | (i << 26) | (imm4 << 16) | (imm3 << 12) | (rd << 8) | imm8);
                } else {
                    error("ASM: MOV immediate out of range");
                }
            }
        } else {
            int rs = parse_reg(&p);
            if (rs < 0) error("ASM: bad Rs for MOV");
            if (rd <= 7 && rs <= 7) {
                asm_emit16(0x4600 | ((rd & 8) << 4) | (rs << 3) | (rd & 7)); // MOV Rd, Rs
            } else {
                asm_emit16(0x4600 | ((rd & 8) << 4) | (rs << 3) | (rd & 7));
            }
        }
        return;
    }

    // ---- MOVT Rd, #imm16 ----
    if (strcmp(mnem, "MOVT") == 0) {
        int rd = parse_reg(&p);
        if (rd < 0) error("ASM: bad Rd for MOVT");
        expect_comma(&p);
        int imm;
        if (!parse_imm(&p, &imm)) error("ASM: expected immediate for MOVT");
        uint16_t imm16 = imm;
        uint32_t i = (imm16 >> 11) & 1;
        uint32_t imm4 = (imm16 >> 12) & 0xF;
        uint32_t imm3 = (imm16 >> 8) & 0x7;
        uint32_t imm8 = imm16 & 0xFF;
        asm_emit32(0xF2C00000 | (i << 26) | (imm4 << 16) | (imm3 << 12) | (rd << 8) | imm8);
        return;
    }

    // ---- ADD, SUB, AND, ORR, EOR, MUL, CMP ----
    // Format: OP Rd, Rs or OP Rd, #imm
    {
        static const struct { const char *name; int op_reg; int op_imm3; int op_imm8; } alu_ops[] = {
            {"ADDS", 0x1800, 0x1C00, 0x3000},  // ADD
            {"ADD",  0x1800, 0x1C00, 0x3000},
            {"SUBS", 0x1A00, 0x1E00, 0x3800},  // SUB
            {"SUB",  0x1A00, 0x1E00, 0x3800},
            {"ANDS", -1, -1, -1},  // handled separately
            {"AND",  -1, -1, -1},
            {"ORRS", -1, -1, -1},
            {"ORR",  -1, -1, -1},
            {"EORS", -1, -1, -1},
            {"EOR",  -1, -1, -1},
            {"MULS", -1, -1, -1},
            {"MUL",  -1, -1, -1},
            {"CMP",  -1, -1, -1},
            {NULL, 0, 0, 0}
        };

        for (int i = 0; alu_ops[i].name; i++) {
            if (strcmp(mnem, alu_ops[i].name) == 0) {
                int rd = parse_reg(&p);
                if (rd < 0) error("ASM: bad Rd");
                expect_comma(&p);

                int imm;
                if (parse_imm(&p, &imm)) {
                    // ADD/SUB with immediate
                    if (strcmp(mnem, "CMP") == 0) {
                        if (rd <= 7 && imm >= 0 && imm <= 255) {
                            asm_emit16(0x2800 | (rd << 8) | (imm & 0xFF));
                        } else error("ASM: CMP immediate out of range");
                    } else if ((mnem[0] == 'A' && mnem[1] == 'D') || (mnem[0] == 'S' && mnem[1] == 'U')) {
                        if (imm >= 0 && imm <= 7) {
                            asm_emit16(alu_ops[i].op_imm3 | (imm << 6) | (rd << 3) | rd);
                        } else if (rd <= 7 && imm >= 0 && imm <= 255) {
                            asm_emit16(alu_ops[i].op_imm8 | (rd << 8) | (imm & 0xFF));
                        } else error("ASM: immediate out of range");
                    } else {
                        error("ASM: immediate not supported for this instruction");
                    }
                } else {
                    int rs = parse_reg(&p);
                    if (rs < 0) error("ASM: bad Rs");

                    // Check for 3rd operand: ADD Rd, Rs, Rn or ADD Rd, Rs, #imm
                    char *saved = p;
                    int has_3rd = expect_comma(&saved);
                    if (has_3rd && (mnem[0] == 'A' || mnem[0] == 'S')) {
                        int imm3;
                        char *try_imm = saved;
                        if (parse_imm(&try_imm, &imm3)) {
                            // ADD Rd, Rs, #imm3 or SUB Rd, Rs, #imm3
                            p = try_imm;
                            if (imm3 >= 0 && imm3 <= 7 && rd <= 7 && rs <= 7) {
                                if (mnem[0] == 'A')
                                    asm_emit16(0x1C00 | (imm3 << 6) | (rs << 3) | rd);
                                else
                                    asm_emit16(0x1E00 | (imm3 << 6) | (rs << 3) | rd);
                            } else error("ASM: 3-operand ADD/SUB immediate out of range");
                            goto alu_done;
                        }
                        int rn = parse_reg(&saved);
                        if (rn >= 0) {
                            // ADD Rd, Rs, Rn or SUB Rd, Rs, Rn
                            p = saved;
                            if (rd <= 7 && rs <= 7 && rn <= 7) {
                                if (mnem[0] == 'A')
                                    asm_emit16(0x1800 | (rn << 6) | (rs << 3) | rd);
                                else
                                    asm_emit16(0x1A00 | (rn << 6) | (rs << 3) | rd);
                            } else error("ASM: 3-operand ADD/SUB requires low registers");
                            goto alu_done;
                        }
                    }

                    // Two-register operations
                    if (strcmp(mnem, "CMP") == 0) {
                        if (rd <= 7 && rs <= 7) asm_emit16(0x4280 | (rs << 3) | rd);
                        else asm_emit16(0x4500 | ((rd & 8) << 4) | (rs << 3) | (rd & 7));
                    } else if (mnem[0] == 'A' && mnem[1] == 'D') {
                        asm_emit16(0x4400 | ((rd & 8) << 4) | (rs << 3) | (rd & 7));
                    } else if (mnem[0] == 'S' && mnem[1] == 'U') {
                        if (rd <= 7 && rs <= 7) asm_emit16(0x1A00 | (rs << 3) | rd);
                        else error("ASM: SUB requires low registers");
                    } else if (strncmp(mnem, "AND", 3) == 0) {
                        asm_emit16(0x4000 | (rs << 3) | rd);
                    } else if (strncmp(mnem, "ORR", 3) == 0) {
                        asm_emit16(0x4300 | (rs << 3) | rd);
                    } else if (strncmp(mnem, "EOR", 3) == 0) {
                        asm_emit16(0x4040 | (rs << 3) | rd);
                    } else if (strncmp(mnem, "MUL", 3) == 0) {
                        asm_emit16(0x4340 | (rs << 3) | rd);
                    }
                }
                alu_done:
                return;
            }
        }
    }

    // ---- LSL, LSR, ASR ----
    if (strcmp(mnem, "LSL") == 0 || strcmp(mnem, "LSLS") == 0) {
        int rd = parse_reg(&p);
        expect_comma(&p);
        int rs = parse_reg(&p);
        expect_comma(&p);
        int imm;
        if (parse_imm(&p, &imm)) {
            asm_emit16(0x0000 | ((imm & 0x1F) << 6) | (rs << 3) | rd);
        } else {
            int rshift = parse_reg(&p);
            asm_emit16(0x4080 | (rshift << 3) | rd);
        }
        return;
    }
    if (strcmp(mnem, "LSR") == 0 || strcmp(mnem, "LSRS") == 0) {
        int rd = parse_reg(&p);
        expect_comma(&p);
        int rs = parse_reg(&p);
        expect_comma(&p);
        int imm;
        if (parse_imm(&p, &imm)) {
            asm_emit16(0x0800 | ((imm & 0x1F) << 6) | (rs << 3) | rd);
        } else error("ASM: LSR register form not implemented");
        return;
    }
    if (strcmp(mnem, "ASR") == 0 || strcmp(mnem, "ASRS") == 0) {
        int rd = parse_reg(&p);
        expect_comma(&p);
        int rs = parse_reg(&p);
        expect_comma(&p);
        int imm;
        if (parse_imm(&p, &imm)) {
            asm_emit16(0x1000 | ((imm & 0x1F) << 6) | (rs << 3) | rd);
        } else error("ASM: ASR register form not implemented");
        return;
    }

    // ---- LDR Rd, [Rs, #offset] / LDR Rd, [Rs] ----
    if (strcmp(mnem, "LDR") == 0) {
        int rd = parse_reg(&p);
        expect_comma(&p);
        p = skip_ws(p);
        if (*p != '[') error("ASM: expected '[' for LDR");
        p++;
        int rb = parse_reg(&p);
        p = skip_ws(p);
        if (*p == ']') {
            p++;
            asm_emit16(0x6800 | (rb << 3) | rd); // LDR Rd, [Rb, #0]
        } else if (*p == ',') {
            p++;
            int imm;
            if (parse_imm(&p, &imm)) {
                p = skip_ws(p);
                if (*p != ']') error("ASM: expected ']'");
                p++;
                if ((imm & 3) || imm < 0 || imm > 124) error("ASM: LDR offset must be 0-124, word-aligned");
                asm_emit16(0x6800 | ((imm >> 2) << 6) | (rb << 3) | rd);
            } else {
                int ro = parse_reg(&p);
                p = skip_ws(p);
                if (*p != ']') error("ASM: expected ']'");
                p++;
                asm_emit16(0x5800 | (ro << 6) | (rb << 3) | rd);
            }
        }
        return;
    }

    // ---- LDRB, LDRH ----
    if (strcmp(mnem, "LDRB") == 0) {
        int rd = parse_reg(&p); expect_comma(&p);
        p = skip_ws(p);
        if (*p != '[') error("ASM: expected '['");
        p++;
        int rb = parse_reg(&p);
        p = skip_ws(p);
        int off = 0;
        if (*p == ',') { p++; parse_imm(&p, &off); }
        p = skip_ws(p);
        if (*p != ']') error("ASM: expected ']'");
        p++;
        asm_emit16(0x7800 | ((off & 0x1F) << 6) | (rb << 3) | rd);
        return;
    }
    if (strcmp(mnem, "LDRH") == 0) {
        int rd = parse_reg(&p); expect_comma(&p);
        p = skip_ws(p);
        if (*p != '[') error("ASM: expected '['");
        p++;
        int rb = parse_reg(&p);
        p = skip_ws(p);
        int off = 0;
        if (*p == ',') { p++; parse_imm(&p, &off); }
        p = skip_ws(p);
        if (*p != ']') error("ASM: expected ']'");
        p++;
        asm_emit16(0x8800 | (((off >> 1) & 0x1F) << 6) | (rb << 3) | rd);
        return;
    }

    // ---- STR Rd, [Rs, #offset] ----
    if (strcmp(mnem, "STR") == 0) {
        int rd = parse_reg(&p); expect_comma(&p);
        p = skip_ws(p);
        if (*p != '[') error("ASM: expected '['");
        p++;
        int rb = parse_reg(&p);
        p = skip_ws(p);
        int off = 0;
        if (*p == ',') { p++; parse_imm(&p, &off); }
        p = skip_ws(p);
        if (*p != ']') error("ASM: expected ']'");
        p++;
        if ((off & 3) || off < 0 || off > 124) error("ASM: STR offset must be 0-124, word-aligned");
        asm_emit16(0x6000 | ((off >> 2) << 6) | (rb << 3) | rd);
        return;
    }

    // ---- STRB, STRH ----
    if (strcmp(mnem, "STRB") == 0) {
        int rd = parse_reg(&p); expect_comma(&p);
        p = skip_ws(p);
        if (*p != '[') error("ASM: expected '['"); p++;
        int rb = parse_reg(&p);
        p = skip_ws(p);
        int off = 0;
        if (*p == ',') { p++; parse_imm(&p, &off); }
        p = skip_ws(p);
        if (*p != ']') error("ASM: expected ']'"); p++;
        asm_emit16(0x7000 | ((off & 0x1F) << 6) | (rb << 3) | rd);
        return;
    }
    if (strcmp(mnem, "STRH") == 0) {
        int rd = parse_reg(&p); expect_comma(&p);
        p = skip_ws(p);
        if (*p != '[') error("ASM: expected '['"); p++;
        int rb = parse_reg(&p);
        p = skip_ws(p);
        int off = 0;
        if (*p == ',') { p++; parse_imm(&p, &off); }
        p = skip_ws(p);
        if (*p != ']') error("ASM: expected ']'"); p++;
        asm_emit16(0x8000 | (((off >> 1) & 0x1F) << 6) | (rb << 3) | rd);
        return;
    }

    // ---- B label / B.cond label ----
    if (mnem[0] == 'B' && (mnem[1] == 0 || parse_condition(mnem + 1) >= 0)) {
        int cond = -1;
        if (mnem[1] != 0) {
            cond = parse_condition(mnem + 1);
            if (cond < 0) error("ASM: unknown condition");
        }

        char label[32];
        if (!parse_label_operand(&p, label)) error("ASM: expected label for branch");

        int target = asm_find_label(label);
        if (target >= 0 && asm_pass == 1) {
            int offset = target - (asm_pos + 4); // PC is current + 4
            if (cond >= 0) {
                // Bcc: 8-bit signed offset (in halfwords)
                offset >>= 1;
                if (offset < -128 || offset > 127) error("ASM: branch offset too large for Bcc");
                asm_emit16(0xD000 | (cond << 8) | (offset & 0xFF));
            } else {
                // B: 11-bit signed offset (in halfwords)
                offset >>= 1;
                if (offset < -1024 || offset > 1023) error("ASM: branch offset too large");
                asm_emit16(0xE000 | (offset & 0x7FF));
            }
        } else {
            // First pass or unresolved - emit placeholder
            if (cond >= 0)
                asm_emit16(0xD000 | (cond << 8)); // Bcc placeholder
            else
                asm_emit16(0xE000); // B placeholder
            // Record fixup
            if (asm_nfixups < ASM_MAX_FIXUPS) {
                asm_fixups[asm_nfixups].offset = asm_pos - 2;
                strncpy(asm_fixups[asm_nfixups].label, label, 31);
                asm_fixups[asm_nfixups].type = (cond >= 0) ? 1 : 0;
                asm_fixups[asm_nfixups].cond = (cond >= 0) ? cond : 0;
                asm_nfixups++;
            }
        }
        return;
    }

    // ---- BL label ---- (32-bit Thumb)
    if (strcmp(mnem, "BL") == 0) {
        char label[32];
        if (!parse_label_operand(&p, label)) error("ASM: expected label for BL");
        // BL uses 32-bit encoding - emit placeholder
        asm_emit32(0xF000D000); // BL placeholder
        if (asm_nfixups < ASM_MAX_FIXUPS) {
            asm_fixups[asm_nfixups].offset = asm_pos - 4;
            strncpy(asm_fixups[asm_nfixups].label, label, 31);
            asm_fixups[asm_nfixups].type = 2; // BL
            asm_nfixups++;
        }
        return;
    }

    // ---- MVN Rd, Rs ----
    if (strcmp(mnem, "MVN") == 0 || strcmp(mnem, "MVNS") == 0) {
        int rd = parse_reg(&p); expect_comma(&p);
        int rs = parse_reg(&p);
        asm_emit16(0x43C0 | (rs << 3) | rd);
        return;
    }

    // ---- NEG Rd, Rs (= RSB Rd, Rs, #0) ----
    if (strcmp(mnem, "NEG") == 0 || strcmp(mnem, "NEGS") == 0) {
        int rd = parse_reg(&p); expect_comma(&p);
        int rs = parse_reg(&p);
        asm_emit16(0x4240 | (rs << 3) | rd);
        return;
    }

    // ---- SVC / SWI #imm ----
    if (strcmp(mnem, "SVC") == 0 || strcmp(mnem, "SWI") == 0) {
        int imm;
        if (!parse_imm(&p, &imm)) error("ASM: expected immediate for SVC");
        asm_emit16(0xDF00 | (imm & 0xFF));
        return;
    }

    // ---- DCD value (emit raw 32-bit word) ----
    if (strcmp(mnem, "DCD") == 0 || strcmp(mnem, "DW") == 0) {
        int val;
        if (!parse_imm(&p, &val)) error("ASM: expected value for DCD");
        // Emit as raw bytes (little-endian)
        if (asm_pass == 1 && asm_pos + 4 <= ASM_MAX_CODE) {
            asm_code[asm_pos] = val & 0xFF;
            asm_code[asm_pos+1] = (val >> 8) & 0xFF;
            asm_code[asm_pos+2] = (val >> 16) & 0xFF;
            asm_code[asm_pos+3] = (val >> 24) & 0xFF;
        }
        asm_pos += 4;
        return;
    }

    // Unknown
    {
        char msg[64];
        snprintf(msg, sizeof(msg), "ASM: unknown instruction '%.20s'", mnem);
        error(msg);
    }
}

// Resolve branch fixups after assembly
static void asm_resolve_fixups(void) {
    for (int i = 0; i < asm_nfixups; i++) {
        int target = asm_find_label(asm_fixups[i].label);
        if (target < 0) {
            char msg[64];
            snprintf(msg, sizeof(msg), "ASM: undefined label '%.20s'", asm_fixups[i].label);
            error(msg);
        }
        int off = asm_fixups[i].offset;
        int pc = off + 4; // PC = instruction address + 4
        int delta = target - pc;

        if (asm_fixups[i].type == 0) {
            // B (unconditional): 11-bit signed offset in halfwords
            delta >>= 1;
            if (delta < -1024 || delta > 1023) error("ASM: branch offset too large");
            uint16_t instr = 0xE000 | (delta & 0x7FF);
            asm_code[off] = instr & 0xFF;
            asm_code[off+1] = (instr >> 8) & 0xFF;
        } else if (asm_fixups[i].type == 1) {
            // Bcc: 8-bit signed offset in halfwords
            delta >>= 1;
            if (delta < -128 || delta > 127) error("ASM: conditional branch offset too large");
            uint16_t instr = 0xD000 | (asm_fixups[i].cond << 8) | (delta & 0xFF);
            asm_code[off] = instr & 0xFF;
            asm_code[off+1] = (instr >> 8) & 0xFF;
        } else if (asm_fixups[i].type == 2) {
            // BL: 32-bit Thumb encoding
            delta >>= 1;
            uint32_t S = (delta < 0) ? 1 : 0;
            uint32_t imm11 = delta & 0x7FF;
            uint32_t imm10 = (delta >> 11) & 0x3FF;
            uint32_t J1 = ((~(delta >> 23)) ^ S) & 1; // simplified
            uint32_t J2 = ((~(delta >> 22)) ^ S) & 1;
            uint32_t hw1 = 0xF000 | (S << 10) | imm10;
            uint32_t hw2 = 0xD000 | (J1 << 13) | (J2 << 11) | imm11;
            asm_code[off] = hw1 & 0xFF;
            asm_code[off+1] = (hw1 >> 8) & 0xFF;
            asm_code[off+2] = hw2 & 0xFF;
            asm_code[off+3] = (hw2 >> 8) & 0xFF;
        }
    }
}

// ============================================================================
// MMBasic command: ASM array%()  ...  END ASM
// ============================================================================
void cmd_asm(void) {
    // Get destination integer array
    int64_t *dest = NULL;
    int size = parseintegerarray(cmdline, &dest, 2, 1, NULL, false, NULL) * 8;

    if (size < 16) error("ASM: array too small");

    // Allocate buffers from temp memory (not static BSS - saves RAM)
    asm_code = (uint8_t *)GetTempMemory(ASM_MAX_CODE);
    asm_labels = (asm_label_t *)GetTempMemory(ASM_MAX_LABELS * sizeof(asm_label_t));
    asm_fixups = (asm_fixup_t *)GetTempMemory(ASM_MAX_FIXUPS * sizeof(asm_fixup_t));
    memset(asm_code, 0, ASM_MAX_CODE);
    asm_nlabels = 0;
    asm_nfixups = 0;

    // Save start position for two-pass assembly
    unsigned char *asm_start = nextstmt;
    unsigned char *asm_end = NULL;

    // Two-pass assembly
    for (asm_pass = 0; asm_pass <= 1; asm_pass++) {
        asm_pos = 0;
        if (asm_pass == 1) asm_nfixups = 0; // keep labels from pass 0

        unsigned char *line_ptr = asm_start;

        while (1) {
            // Skip zero bytes (line terminators) and whitespace
            while (*line_ptr == 0) line_ptr++;
            if (*line_ptr == 0xFF)
                error("ASM: missing END ASM");

            // Skip T_NEWLINE + T_LINENBR
            while (*line_ptr == T_NEWLINE) line_ptr++;
            if (*line_ptr == T_LINENBR) line_ptr += 3;

            // Detoken this line
            char linebuf[256];
            unsigned char *next = llist((unsigned char *)linebuf, line_ptr);

            // Check for END ASM
            char *trimmed = linebuf;
            while (*trimmed == ' ') trimmed++;

            bool is_end = false;
            if (strncasecmp(trimmed, "END", 3) == 0) {
                char *after = trimmed + 3;
                while (*after == ' ') after++;
                if (strncasecmp(after, "ASM", 3) == 0 || *after == 0)
                    is_end = true;
            }
            if (is_end) {
                asm_end = next;
                break;
            }

            asm_line(trimmed);
            line_ptr = next;
        }
    }

    // Set nextstmt past END ASM
    if (asm_end) nextstmt = asm_end;

    // Resolve any remaining fixups
    asm_resolve_fixups();

    if (asm_pos > size) error("ASM: code too large for array");

    // Copy code to integer array
    // First element = code size, rest = executable code
    // Set bit 0 of address for Thumb mode
    memcpy(dest, asm_code, asm_pos);

    // Flush instruction cache
    __dsb();
    __isb();

    char msg[32];
    sprintf(msg, "%d bytes assembled\r\n", asm_pos);
    MMPrintString(msg);
}

// Function to execute assembled code: USR(addr [, arg])
// Returns the value in R0 after the code runs
// The code address must have bit 0 set for Thumb mode
void fun_usr(void) {
    int64_t addr, arg = 0;
    getargs(&ep, 3, (unsigned char *)",");
    addr = getinteger(argv[0]);
    if (argc >= 3) arg = getinteger(argv[2]);

    // Ensure Thumb bit is set
    addr |= 1;

    // Call the function: takes one int32 arg in R0, returns int32 in R0
    // (assembly code operates on 32-bit ARM registers)
    typedef uint32_t (*asm_func_t)(uint32_t);
    asm_func_t func = (asm_func_t)addr;
    iret = (int64_t)(int32_t)func((uint32_t)arg);  // sign-extend 32-bit result
    targ = T_INT;
}

#endif // ADAFRUIT_FRUIT_JAM
