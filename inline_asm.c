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

// Parse VFP single-precision register: S0-S31
static int parse_sreg(char **pp) {
    char *p = skip_ws(*pp);
    if ((*p == 'S' || *p == 's') && isdigit(p[1])) {
        int reg = atoi(p + 1);
        p += 2;
        if (reg >= 10) p++;
        if (reg > 31) return -1;
        *pp = p;
        return reg;
    }
    return -1;
}

// Parse immediate: #expression
// Supports: #42, #0xFF, #-5, #myvar%, #PEEK(VARADDR x%), #(2+3)*4
// On pass 0, tries simple parse; on pass 1, evaluates BASIC expressions
static int parse_imm(char **pp, int *val) {
    char *p = skip_ws(*pp);
    if (*p != '#') return 0;
    p++;

    // Try simple numeric literal first (fast path)
    char *endp;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
        *val = strtol(p, &endp, 16);
        if (endp > p) { *pp = endp; return 1; }
    } else if (p[0] == '-' || isdigit(p[0])) {
        *val = strtol(p, &endp, 10);
        if (endp > p && !isalpha(*endp) && *endp != '_' && *endp != '(') { *pp = endp; return 1; }
    }

    // Not a simple literal - try BASIC variable or expression
    // But NOT if it looks like a register name (R0-R15, SP, LR, PC, S0-S31)
    if ((*p == 'R' || *p == 'r') && isdigit(p[1])) return 0;
    if ((*p == 'S' || *p == 's') && isdigit(p[1])) return 0;
    if (strncasecmp(p, "SP", 2) == 0 && !isalnum(p[2])) return 0;
    if (strncasecmp(p, "LR", 2) == 0 && !isalnum(p[2])) return 0;
    if (strncasecmp(p, "PC", 2) == 0 && !isalnum(p[2])) return 0;
    {
        char *start = p;
        char name[64];
        int ni = 0;
        while (*p && (isalnum(*p) || *p == '_' || *p == '.' || *p == '%' || *p == '$' || *p == '!' || *p == '(' || *p == ')')) {
            if (ni < 63) name[ni++] = *p;
            p++;
        }
        name[ni] = 0;

        if (ni > 0) {
            // Try to find as BASIC variable (return NULL if not found)
            void *ptr = findvar((unsigned char *)name, V_NOFIND_NULL);
            if (ptr) {
                // Determine type from name suffix: % = integer, ! or none = float
                // findvar returns pointer to the variable's data
                bool is_int = false;
                for (int k = ni - 1; k >= 0; k--) {
                    if (name[k] == '%') { is_int = true; break; }
                    if (name[k] == '!' || name[k] == '$') break;
                    if (!isalnum(name[k]) && name[k] != '_') break;
                }
                if (is_int) {
                    *val = (int)(*(long long int *)ptr);
                } else {
                    *val = (int)(*(MMFLOAT *)ptr);
                }

                // Check for simple arithmetic: + - * /
                p = skip_ws(p);
                while (*p == '+' || *p == '-' || *p == '*' || *p == '/') {
                    char op = *p++;
                    p = skip_ws(p);
                    int rhs;
                    if (isdigit(*p) || *p == '-') {
                        rhs = strtol(p, &p, 0);
                    } else {
                        // Try another variable
                        char name2[64];
                        int ni2 = 0;
                        while (*p && (isalnum(*p) || *p == '_' || *p == '%' || *p == '!' || *p == '(' || *p == ')')) {
                            if (ni2 < 63) name2[ni2++] = *p;
                            p++;
                        }
                        name2[ni2] = 0;
                        void *ptr2 = findvar((unsigned char *)name2, V_NOFIND_NULL);
                        if (ptr2) {
                            bool is_int2 = (strchr(name2, '%') != NULL);
                            if (is_int2) rhs = (int)(*(long long int *)ptr2);
                            else rhs = (int)(*(MMFLOAT *)ptr2);
                        } else {
                            rhs = 0;
                        }
                    }
                    if (op == '+') *val += rhs;
                    else if (op == '-') *val -= rhs;
                    else if (op == '*') *val *= rhs;
                    else if (op == '/' && rhs != 0) *val /= rhs;
                    p = skip_ws(p);
                }

                *pp = p;
                return 1;
            }
        }
    }

    // Could not parse
    return 0;
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

    // ---- ADD SP, SP, #imm / SUB SP, SP, #imm (stack adjust) ----
    if ((strcmp(mnem, "ADD") == 0 || strcmp(mnem, "ADDS") == 0 ||
         strcmp(mnem, "SUB") == 0 || strcmp(mnem, "SUBS") == 0)) {
        char *saved_p = p;
        int rd = parse_reg(&p);
        if (rd == 13) { // SP
            expect_comma(&p);
            char *saved2 = p;
            int rs = parse_reg(&p);
            if (rs == 13) { // ADD SP, SP, #imm
                expect_comma(&p);
                int imm;
                if (parse_imm(&p, &imm) && (imm & 3) == 0 && imm >= 0 && imm <= 508) {
                    if (mnem[0] == 'A')
                        asm_emit16(0xB000 | ((imm >> 2) & 0x7F));
                    else
                        asm_emit16(0xB080 | ((imm >> 2) & 0x7F));
                    return;
                }
            }
            p = saved2;
            // ADD SP, #imm (2-operand shorthand)
            int imm;
            if (parse_imm(&p, &imm) && (imm & 3) == 0 && imm >= 0 && imm <= 508) {
                if (mnem[0] == 'A')
                    asm_emit16(0xB000 | ((imm >> 2) & 0x7F));
                else
                    asm_emit16(0xB080 | ((imm >> 2) & 0x7F));
                return;
            }
        }
        p = saved_p; // fall through to general ALU
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
                            } else if (rd == rs && rd <= 7 && imm3 >= 0 && imm3 <= 255) {
                                // Fallback: ADD/SUB Rd, #imm8 (2-operand)
                                if (mnem[0] == 'A')
                                    asm_emit16(0x3000 | (rd << 8) | (imm3 & 0xFF));
                                else
                                    asm_emit16(0x3800 | (rd << 8) | (imm3 & 0xFF));
                            } else {
                                char msg[80];
                                snprintf(msg, sizeof(msg), "ASM: ADD/SUB imm out of range (rd=%d rs=%d imm=%d)", rd, rs, imm3);
                                error(msg);
                            }
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
        if (*p == ']') {
            p++;
            asm_emit16(0x7800 | (rb << 3) | rd);
        } else if (*p == ',') {
            p++;
            int imm;
            if (parse_imm(&p, &imm)) {
                p = skip_ws(p); if (*p != ']') error("ASM: expected ']'"); p++;
                asm_emit16(0x7800 | ((imm & 0x1F) << 6) | (rb << 3) | rd);
            } else {
                int ro = parse_reg(&p);
                p = skip_ws(p); if (*p != ']') error("ASM: expected ']'"); p++;
                asm_emit16(0x5C00 | (ro << 6) | (rb << 3) | rd);
            }
        }
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
        if (*p == ']') {
            p++;
            asm_emit16(0x7000 | (rb << 3) | rd);
        } else if (*p == ',') {
            p++;
            int imm;
            if (parse_imm(&p, &imm)) {
                p = skip_ws(p); if (*p != ']') error("ASM: expected ']'"); p++;
                asm_emit16(0x7000 | ((imm & 0x1F) << 6) | (rb << 3) | rd);
            } else {
                int ro = parse_reg(&p);
                p = skip_ws(p); if (*p != ']') error("ASM: expected ']'"); p++;
                asm_emit16(0x5400 | (ro << 6) | (rb << 3) | rd);
            }
        }
        return;
    }
    if (strcmp(mnem, "STRH") == 0) {
        int rd = parse_reg(&p); expect_comma(&p);
        p = skip_ws(p);
        if (*p != '[') error("ASM: expected '['"); p++;
        int rb = parse_reg(&p);
        p = skip_ws(p);
        if (*p == ']') {
            p++;
            asm_emit16(0x8000 | (rb << 3) | rd);
        } else if (*p == ',') {
            p++;
            int imm;
            if (parse_imm(&p, &imm)) {
                p = skip_ws(p); if (*p != ']') error("ASM: expected ']'"); p++;
                asm_emit16(0x8000 | (((imm >> 1) & 0x1F) << 6) | (rb << 3) | rd);
            } else {
                int ro = parse_reg(&p);
                p = skip_ws(p); if (*p != ']') error("ASM: expected ']'"); p++;
                asm_emit16(0x5200 | (ro << 6) | (rb << 3) | rd);
            }
        }
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
                // Bcc narrow: 8-bit signed offset in halfwords
                offset >>= 1;
                if (offset < -128 || offset > 127) error("ASM: conditional branch too far (keep code short or use B)");
                asm_emit16(0xD000 | (cond << 8) | (offset & 0xFF));
            } else {
                // B: 11-bit signed offset (in halfwords)
                offset >>= 1;
                if (offset < -1024 || offset > 1023) error("ASM: branch offset too large");
                asm_emit16(0xE000 | (offset & 0x7FF));
            }
        } else {
            if (cond >= 0)
                asm_emit16(0xD000 | (cond << 8)); // Bcc placeholder (narrow)
            else
                asm_emit16(0xE000); // B placeholder
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

    // ---- Data directives ----
    // DCD / EQUD - emit 32-bit word
    if (strcmp(mnem, "DCD") == 0 || strcmp(mnem, "EQUD") == 0 || strcmp(mnem, "DW") == 0) {
        int val;
        if (!parse_imm(&p, &val)) error("ASM: expected value");
        if (asm_pass == 1 && asm_pos + 4 <= ASM_MAX_CODE) {
            asm_code[asm_pos] = val & 0xFF;
            asm_code[asm_pos+1] = (val >> 8) & 0xFF;
            asm_code[asm_pos+2] = (val >> 16) & 0xFF;
            asm_code[asm_pos+3] = (val >> 24) & 0xFF;
        }
        asm_pos += 4;
        return;
    }
    // EQUW - emit 16-bit halfword
    if (strcmp(mnem, "EQUW") == 0 || strcmp(mnem, "DCW") == 0) {
        int val;
        if (!parse_imm(&p, &val)) error("ASM: expected value");
        if (asm_pass == 1 && asm_pos + 2 <= ASM_MAX_CODE) {
            asm_code[asm_pos] = val & 0xFF;
            asm_code[asm_pos+1] = (val >> 8) & 0xFF;
        }
        asm_pos += 2;
        return;
    }
    // EQUB / DCB - emit byte
    if (strcmp(mnem, "EQUB") == 0 || strcmp(mnem, "DCB") == 0) {
        int val;
        if (!parse_imm(&p, &val)) error("ASM: expected value");
        if (asm_pass == 1 && asm_pos < ASM_MAX_CODE) {
            asm_code[asm_pos] = val & 0xFF;
        }
        asm_pos += 1;
        return;
    }
    // EQUS - emit string bytes
    if (strcmp(mnem, "EQUS") == 0) {
        p = skip_ws(p);
        if (*p != '"') error("ASM: expected string for EQUS");
        p++;
        while (*p && *p != '"') {
            if (asm_pass == 1 && asm_pos < ASM_MAX_CODE)
                asm_code[asm_pos] = *p;
            asm_pos++;
            p++;
        }
        if (*p == '"') p++;
        return;
    }
    // ALIGN - align to 4-byte boundary
    if (strcmp(mnem, "ALIGN") == 0) {
        while (asm_pos & 3) {
            if (asm_pass == 1 && asm_pos < ASM_MAX_CODE)
                asm_code[asm_pos] = 0;
            asm_pos++;
        }
        return;
    }

    // ============================================================================
    // VFP/FPU Instructions (Cortex-M33 single-precision)
    // ============================================================================

    // Helper: encode Sd (destination single reg) into VFP instruction
    // Sd is encoded as D:Vd where D=Sd[0], Vd=Sd[4:1]
    #define VFP_SD(sd) ((((sd) >> 1) << 12) | (((sd) & 1) << 22))
    #define VFP_SN(sn) ((((sn) >> 1) << 16) | (((sn) & 1) << 7))
    #define VFP_SM(sm) ((((sm) >> 1) << 0)  | (((sm) & 1) << 5))

    // ---- VADD.F32 Sd, Sn, Sm ----
    if (strcmp(mnem, "VADD") == 0 || strcmp(mnem, "VADDF32") == 0) {
        // Skip optional .F32 suffix
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        int sn = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sn < 0 || sm < 0) error("ASM: bad register for VADD.F32");
        asm_emit32(0xEE300A00 | VFP_SD(sd) | VFP_SN(sn) | VFP_SM(sm));
        return;
    }

    // ---- VSUB.F32 Sd, Sn, Sm ----
    if (strcmp(mnem, "VSUB") == 0 || strcmp(mnem, "VSUBF32") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        int sn = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sn < 0 || sm < 0) error("ASM: bad register for VSUB.F32");
        asm_emit32(0xEE300A40 | VFP_SD(sd) | VFP_SN(sn) | VFP_SM(sm));
        return;
    }

    // ---- VMUL.F32 Sd, Sn, Sm ----
    if (strcmp(mnem, "VMUL") == 0 || strcmp(mnem, "VMULF32") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        int sn = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sn < 0 || sm < 0) error("ASM: bad register for VMUL.F32");
        asm_emit32(0xEE200A00 | VFP_SD(sd) | VFP_SN(sn) | VFP_SM(sm));
        return;
    }

    // ---- VDIV.F32 Sd, Sn, Sm ----
    if (strcmp(mnem, "VDIV") == 0 || strcmp(mnem, "VDIVF32") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        int sn = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sn < 0 || sm < 0) error("ASM: bad register for VDIV.F32");
        asm_emit32(0xEE800A00 | VFP_SD(sd) | VFP_SN(sn) | VFP_SM(sm));
        return;
    }

    // ---- VNEG.F32 Sd, Sm ----
    if (strcmp(mnem, "VNEG") == 0 || strcmp(mnem, "VNEGF32") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sm < 0) error("ASM: bad register for VNEG.F32");
        asm_emit32(0xEEB10A40 | VFP_SD(sd) | VFP_SM(sm));
        return;
    }

    // ---- VABS.F32 Sd, Sm ----
    if (strcmp(mnem, "VABS") == 0 || strcmp(mnem, "VABSF32") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sm < 0) error("ASM: bad register for VABS.F32");
        asm_emit32(0xEEB00AC0 | VFP_SD(sd) | VFP_SM(sm));
        return;
    }

    // ---- VSQRT.F32 Sd, Sm ----
    if (strcmp(mnem, "VSQRT") == 0 || strcmp(mnem, "VSQRTF32") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sm < 0) error("ASM: bad register for VSQRT.F32");
        asm_emit32(0xEEB10AC0 | VFP_SD(sd) | VFP_SM(sm));
        return;
    }

    // ---- VMOV Sd, Sm  or  VMOV Sd, Rn  or  VMOV Rd, Sn ----
    if (strcmp(mnem, "VMOV") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        char *saved = p;
        int sd = parse_sreg(&p);
        if (sd >= 0) {
            expect_comma(&p);
            // VMOV Sd, Sm or VMOV Sd, Rn
            char *saved2 = p;
            int sm = parse_sreg(&p);
            if (sm >= 0) {
                // VMOV.F32 Sd, Sm (copy)
                asm_emit32(0xEEB00A40 | VFP_SD(sd) | VFP_SM(sm));
            } else {
                p = saved2;
                int rn = parse_reg(&p);
                if (rn >= 0) {
                    // VMOV Sd, Rn (ARM to VFP)
                    asm_emit32(0xEE000A10 | (rn << 12) | VFP_SN(sd));
                } else {
                    error("ASM: bad operand for VMOV");
                }
            }
        } else {
            p = saved;
            int rd = parse_reg(&p);
            if (rd >= 0) {
                expect_comma(&p);
                int sn = parse_sreg(&p);
                if (sn >= 0) {
                    // VMOV Rd, Sn (VFP to ARM)
                    asm_emit32(0xEE100A10 | (rd << 12) | VFP_SN(sn));
                } else {
                    error("ASM: bad operand for VMOV");
                }
            } else {
                error("ASM: bad operand for VMOV");
            }
        }
        return;
    }

    // ---- VCMP.F32 Sd, Sm ----
    if (strcmp(mnem, "VCMP") == 0 || strcmp(mnem, "VCMPF32") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        // Check for #0 (compare against zero)
        int imm;
        if (parse_imm(&p, &imm) && imm == 0) {
            asm_emit32(0xEEB50A40 | VFP_SD(sd));
        } else {
            int sm = parse_sreg(&p);
            if (sm < 0) error("ASM: bad register for VCMP.F32");
            asm_emit32(0xEEB40A40 | VFP_SD(sd) | VFP_SM(sm));
        }
        return;
    }

    // ---- VMRS APSR_nzcv, FPSCR (move FP flags to ARM condition flags) ----
    if (strcmp(mnem, "VMRS") == 0) {
        // VMRS APSR_nzcv, FPSCR
        asm_emit32(0xEEF1FA10);
        return;
    }

    // ---- VCVT: int<->float conversions ----
    // VCVT.F32.S32 Sd, Sm  (signed int to float)
    // VCVT.S32.F32 Sd, Sm  (float to signed int)
    // VCVT.F32.U32 Sd, Sm  (unsigned int to float)
    // VCVT.U32.F32 Sd, Sm  (float to unsigned int)
    if (strcmp(mnem, "VCVT") == 0) {
        if (*p == '.') p++;
        // Parse conversion type
        bool to_float = false, is_signed = true;
        if (strncasecmp(p, "F32", 3) == 0) {
            to_float = true;
            p += 3;
            if (*p == '.') p++;
            if (strncasecmp(p, "S32", 3) == 0) { is_signed = true; p += 3; }
            else if (strncasecmp(p, "U32", 3) == 0) { is_signed = false; p += 3; }
            else error("ASM: VCVT expected .S32 or .U32");
        } else if (strncasecmp(p, "S32", 3) == 0) {
            to_float = false; is_signed = true; p += 3;
            if (*p == '.') p++;
            if (strncasecmp(p, "F32", 3) == 0) p += 3;
            else error("ASM: VCVT expected .F32");
        } else if (strncasecmp(p, "U32", 3) == 0) {
            to_float = false; is_signed = false; p += 3;
            if (*p == '.') p++;
            if (strncasecmp(p, "F32", 3) == 0) p += 3;
            else error("ASM: VCVT expected .F32");
        } else error("ASM: bad VCVT type");

        int sd = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sm < 0) error("ASM: bad register for VCVT");

        if (to_float) {
            // int->float: VCVT.F32.S32 = EEB80AC0, VCVT.F32.U32 = EEB80A40
            asm_emit32((is_signed ? 0xEEB80AC0 : 0xEEB80A40) | VFP_SD(sd) | VFP_SM(sm));
        } else {
            // float->int: VCVT.S32.F32 = EEBD0AC0, VCVT.U32.F32 = EEBC0AC0
            asm_emit32((is_signed ? 0xEEBD0AC0 : 0xEEBC0AC0) | VFP_SD(sd) | VFP_SM(sm));
        }
        return;
    }

    // ---- VLDR Sd, [Rn, #offset]  (load float from memory) ----
    if (strcmp(mnem, "VLDR") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        p = skip_ws(p);
        if (*p != '[') error("ASM: expected '[' for VLDR");
        p++;
        int rn = parse_reg(&p);
        if (rn < 0) error("ASM: bad base register for VLDR");
        p = skip_ws(p);
        int off = 0;
        if (*p == ',') { p++; parse_imm(&p, &off); }
        p = skip_ws(p);
        if (*p != ']') error("ASM: expected ']'");
        p++;
        if ((off & 3) || off < -1020 || off > 1020) error("ASM: VLDR offset must be word-aligned, -1020..1020");
        uint32_t U = (off >= 0) ? 1 : 0;
        uint32_t imm8 = (off >= 0 ? off : -off) >> 2;
        asm_emit32(0xED100A00 | (U << 23) | VFP_SD(sd) | (rn << 16) | (imm8 & 0xFF));
        return;
    }

    // ---- VSTR Sd, [Rn, #offset]  (store float to memory) ----
    if (strcmp(mnem, "VSTR") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        p = skip_ws(p);
        if (*p != '[') error("ASM: expected '[' for VSTR");
        p++;
        int rn = parse_reg(&p);
        if (rn < 0) error("ASM: bad base register for VSTR");
        p = skip_ws(p);
        int off = 0;
        if (*p == ',') { p++; parse_imm(&p, &off); }
        p = skip_ws(p);
        if (*p != ']') error("ASM: expected ']'");
        p++;
        if ((off & 3) || off < -1020 || off > 1020) error("ASM: VSTR offset must be word-aligned, -1020..1020");
        uint32_t U = (off >= 0) ? 1 : 0;
        uint32_t imm8 = (off >= 0 ? off : -off) >> 2;
        asm_emit32(0xED000A00 | (U << 23) | VFP_SD(sd) | (rn << 16) | (imm8 & 0xFF));
        return;
    }

    // ---- VPUSH/VPOP {Sd-Sn} ----
    if (strcmp(mnem, "VPUSH") == 0) {
        p = skip_ws(p);
        if (*p != '{') error("ASM: expected '{' for VPUSH");
        p++;
        int s1 = parse_sreg(&p);
        p = skip_ws(p);
        int count = 1;
        if (*p == '-') { p++; int s2 = parse_sreg(&p); count = s2 - s1 + 1; }
        p = skip_ws(p);
        if (*p != '}') error("ASM: expected '}'"); p++;
        if (s1 < 0 || count < 1 || count > 16) error("ASM: bad VPUSH register range");
        asm_emit32(0xED2D0A00 | VFP_SD(s1) | count);
        return;
    }
    if (strcmp(mnem, "VPOP") == 0) {
        p = skip_ws(p);
        if (*p != '{') error("ASM: expected '{' for VPOP");
        p++;
        int s1 = parse_sreg(&p);
        p = skip_ws(p);
        int count = 1;
        if (*p == '-') { p++; int s2 = parse_sreg(&p); count = s2 - s1 + 1; }
        p = skip_ws(p);
        if (*p != '}') error("ASM: expected '}'"); p++;
        if (s1 < 0 || count < 1 || count > 16) error("ASM: bad VPOP register range");
        asm_emit32(0xECBD0A00 | VFP_SD(s1) | count);
        return;
    }

    // ---- VMLA.F32 Sd, Sn, Sm  (multiply-accumulate: Sd = Sd + Sn*Sm) ----
    if (strcmp(mnem, "VMLA") == 0 || strcmp(mnem, "VMLAF32") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        int sn = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sn < 0 || sm < 0) error("ASM: bad register for VMLA.F32");
        asm_emit32(0xEE000A00 | VFP_SD(sd) | VFP_SN(sn) | VFP_SM(sm));
        return;
    }

    // ---- VMLS.F32 Sd, Sn, Sm  (multiply-subtract: Sd = Sd - Sn*Sm) ----
    if (strcmp(mnem, "VMLS") == 0 || strcmp(mnem, "VMLSF32") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        int sn = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sn < 0 || sm < 0) error("ASM: bad register for VMLS.F32");
        asm_emit32(0xEE000A40 | VFP_SD(sd) | VFP_SN(sn) | VFP_SM(sm));
        return;
    }

    // ---- VNMUL.F32 Sd, Sn, Sm  (negate-multiply: Sd = -(Sn*Sm)) ----
    if (strcmp(mnem, "VNMUL") == 0) {
        if (*p == '.') { while (*p && *p != ' ' && *p != '\t') p++; }
        int sd = parse_sreg(&p); expect_comma(&p);
        int sn = parse_sreg(&p); expect_comma(&p);
        int sm = parse_sreg(&p);
        if (sd < 0 || sn < 0 || sm < 0) error("ASM: bad register for VNMUL.F32");
        asm_emit32(0xEE200A40 | VFP_SD(sd) | VFP_SN(sn) | VFP_SM(sm));
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

// Evaluate a BASIC expression and determine its type
// Returns: 0 = integer, 1 = float
static int eval_asm_arg_typed(unsigned char *expr, uint32_t *ival, float *fval) {
    // Check if expression looks like an integer: variable with %, literal without .
    // Use getnumber to evaluate, then check if it has a fractional part
    MMFLOAT f = getnumber(expr);

    // Heuristic: integer if no fractional part AND the expression contains %
    // or is a plain integer literal
    bool looks_integer = false;
    char *s = (char *)expr;
    while (*s == ' ') s++;

    // Check for integer variable (has %) or integer literal (digits only, no .)
    if (strchr(s, '%')) {
        looks_integer = true;
    } else {
        // Check if it's a plain number without decimal point
        char *p = s;
        if (*p == '-') p++;
        if (*p == '&') looks_integer = true; // hex literal
        else if (isdigit(*p)) {
            bool has_dot = false;
            while (*p && (isdigit(*p) || *p == '.')) {
                if (*p == '.') has_dot = true;
                p++;
            }
            if (!has_dot && (*p == 0 || *p == ')' || *p == ',' || *p == ' '))
                looks_integer = true;
        }
    }

    if (looks_integer) {
        *ival = (uint32_t)(int32_t)f;
        *fval = 0;
        return 0; // integer
    } else {
        *fval = (float)f;
        *ival = 0;
        return 1; // float
    }
}

// Resolve ASM code address from first argument
static uint32_t resolve_asm_addr(unsigned char *arg) {
    char *s = (char *)arg;
    bool is_array = false;
    int depth = 0;
    for (char *c = s; *c; c++) {
        if (*c == '(') depth++;
        if (*c == ')') { depth--; if (depth == 0 && (c[1] == 0 || c[1] == ' ')) { is_array = true; break; } }
    }
    if (is_array) {
        int64_t *arrptr = NULL;
        parseintegerarray(arg, &arrptr, 1, 1, NULL, false, NULL);
        return (uint32_t)(uintptr_t)arrptr | 1;
    }
    return (uint32_t)getinteger(arg) | 1;
}

// Call ASM with auto type routing:
// - Integer args go to R0, R1, R2, R3 (in order)
// - Float args are collected into a float[] block, pointer passed as next R arg
// - ASM receives: integers in R registers, float block pointer in next R
// Example: CALL code%(), fb%, cx, xr, cy, yr
//   → R0=fb%, R1=pointer to {cx, xr, cy, yr} as float32 array
//   ASM does: VLDR S0, [R1] / VLDR S1, [R1, #4] / etc.
static uint32_t do_asm_call(unsigned char **argv, int argc) {
    uint32_t addr = resolve_asm_addr(argv[0]);

    uint32_t r[4] = {0, 0, 0, 0};
    static float fbuf[8]; // float buffer (static to survive the call)
    int ri = 0, fi = 0;
    bool has_floats = false;

    // First pass: separate ints and floats
    for (int i = 1; i < 9 && (i * 2) < argc; i++) {
        uint32_t ival;
        float fval;
        int is_float = eval_asm_arg_typed(argv[i * 2], &ival, &fval);
        if (is_float) {
            if (fi < 8) fbuf[fi++] = fval;
            has_floats = true;
        } else {
            if (ri < 4) r[ri++] = ival;
        }
    }

    // If there are floats, pass float buffer pointer as the next R arg
    if (has_floats && ri < 4) {
        r[ri++] = (uint32_t)(uintptr_t)fbuf;
    }

    typedef uint32_t (*asm_func4_t)(uint32_t, uint32_t, uint32_t, uint32_t);
    return ((asm_func4_t)addr)(r[0], r[1], r[2], r[3]);
}

// CALL code%(), args...  - call ASM, discard return
void cmd_callasm(void) {
    getargs(&cmdline, 17, (unsigned char *)","); // up to 8 args
    if (argc < 1) error("Syntax");
    do_asm_call(argv, argc);
}

// USR(code%(), args...) - call ASM, return R0 as integer
void fun_usr(void) {
    getargs(&ep, 17, (unsigned char *)",");
    if (argc < 1) error("Syntax");
    iret = (int64_t)(int32_t)do_asm_call(argv, argc);
    targ = T_INT;
}

#endif // ADAFRUIT_FRUIT_JAM
