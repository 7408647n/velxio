#!/usr/bin/env python3
"""
List the register bits Arduino libraries wait on, so a model knows what it
has to let settle (README.md next to this file, "Polled bits").

A driver that waits for a chip does it one of three ways, and a register
model that stores every written byte verbatim hangs all three:

    while (read8(REG_STATUS) & STATUS_MEASURING) delay(1);
    do { v = readRegister(REG_CTRL); } while (v & CTRL_RESET);
    Adafruit_BusIO_RegisterBits reset = Adafruit_BusIO_RegisterBits(&pwr, 1, 7);
    while (reset.read() == 1) delay(1);

This script finds those loops in library sources and prints, per library,
source line and register, the register and mask the loop waits on, whether
it waits for the bits to clear or to be set, and whether the loop gives up
on its own (a timeout or a retry count) or waits for ever.

It reads source text with regular expressions and a brace matcher: it is a
lint, not a compiler. Register and mask names are resolved from the
#defines, enums and constants of the same library; what it cannot resolve
is printed as the name.

Usage:
    lint-polled-bits.py [--markdown | --json] FOLDER [FOLDER ...]

Each FOLDER is a folder of Arduino libraries (one library per child folder,
as the app container's /var/velxio/libcache keeps them) or one library.
"""

import argparse
import json
import os
import re
import sys

SOURCE_EXT = ('.c', '.cpp', '.cc', '.h', '.hpp')
MAX_SOURCE_BYTES = 1 << 20
SKIP_DIRS = {'examples', 'extras', 'test', 'tests', 'docs', '.git'}

# Calls that read something other than a chip register.
NOT_A_REGISTER_READ = re.compile(
    r'^(digitalRead|analogRead|digitalReadFast|gpio_get_level|gpio_get|Serial\d?\.\w+|'
    r'\w*[Ss]erial\w*\.read|\w*\.available|millis|micros|readBytesUntil|readString\w*|'
    r'\w*[Ff]ile\w*\.read|client\.read|\w*[Ss]tream\w*\.read|\w*\.readBytes|'
    r'pgm_read_\w+|read_digital|readPin|readSensorPin|\w*spi\w*\.transfer)$')

# A call whose name says it reads (read8, readRegister, i2cread, getStatus...).
READ_CALL = re.compile(
    r'(?P<name>[A-Za-z_]\w*(?:(?:\.|->|::)[A-Za-z_]\w*)*)\s*\(')
READISH = re.compile(r'(read|Read|READ|Rd[A-Z]|get[A-Z_]\w*[Ss]tatus|getStatus|status\b|Status\b|_rd\b|rd_?reg)')

BOUNDED = re.compile(
    r'(millis\s*\(|micros\s*\(|[Tt]ime[Oo]ut|[Tt]imeout|\b\w*(?:retr|tr[iy]|count|cnt|attempt|loops?|'
    r'tries|n|i|j|k)\w*\s*(?:--|\+\+|<|>|<=|>=)|--\s*\w|\+\+\s*\w|\bbreak\b|\breturn\b)')

NUMBER = re.compile(r'^(0[xX][0-9a-fA-F]+|0[bB][01]+|\d+)[uUlL]*$')


COMMENT_OR_STRING = re.compile(
    r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'', re.S)


def strip_comments(text: str) -> str:
    """Comments and the inside of string literals blanked, newlines kept, so
    offsets and line numbers stay where they were."""
    def blank(m):
        t = m.group(0)
        if t[0] in '"\'':
            return t[0] + re.sub(r'[^\n]', ' ', t[1:-1]) + t[-1]
        return re.sub(r'[^\n]', ' ', t)
    return COMMENT_OR_STRING.sub(blank, text)


def balanced(text: str, start: int, open_c: str, close_c: str):
    """The end (exclusive) of the bracketed group opening at text[start]."""
    depth = 0
    for i in range(start, len(text)):
        if text[i] == open_c:
            depth += 1
        elif text[i] == close_c:
            depth -= 1
            if depth == 0:
                return i + 1
    return None


def split_args(args: str) -> list:
    out, depth, cur = [], 0, []
    for c in args:
        if c in '([{':
            depth += 1
        elif c in ')]}':
            depth -= 1
        if c == ',' and depth == 0:
            out.append(''.join(cur).strip())
            cur = []
        else:
            cur.append(c)
    if ''.join(cur).strip():
        out.append(''.join(cur).strip())
    return out


def to_int(token: str):
    token = token.strip().strip('()').strip()
    m = NUMBER.match(token)
    if not m:
        return None
    t = m.group(1)
    if t[:2] in ('0x', '0X'):
        return int(t, 16)
    if t[:2] in ('0b', '0B'):
        return int(t[2:], 2)
    return int(t)


class Library:
    """The sources of one library and the constants they define."""

    DEFINE = re.compile(r'^[ \t]*#[ \t]*define[ \t]+([A-Za-z_]\w*)[ \t]+(.+?)[ \t]*$', re.M)
    ASSIGN = re.compile(r'\b([A-Za-z_]\w*)\s*=\s*(\(?\s*(?:0[xX][0-9a-fA-F]+|0[bB][01]+|\d+)[uUlL]*\s*\)?)\s*[,;}]')

    def __init__(self, name: str, root: str):
        self.name = name
        self.root = root
        self.files = {}
        for folder, dirs, files in os.walk(root):
            dirs[:] = [d for d in dirs if d.lower() not in SKIP_DIRS]
            for f in files:
                path = os.path.join(folder, f)
                # Font and image tables run to megabytes and hold no driver code.
                if f.endswith(SOURCE_EXT) and os.path.getsize(path) <= MAX_SOURCE_BYTES:
                    try:
                        with open(path, encoding='utf-8', errors='replace') as fh:
                            self.files[os.path.relpath(path, root)] = strip_comments(fh.read())
                    except OSError:
                        pass
        self.consts = {}
        for text in self.files.values():
            for m in self.DEFINE.finditer(text):
                self.consts.setdefault(m.group(1), m.group(2))
            for m in self.ASSIGN.finditer(text):
                self.consts.setdefault(m.group(1), m.group(2))

    def value(self, expr: str, depth: int = 0):
        """An expression of constants as a number, or None."""
        expr = expr.strip()
        if depth > 8 or not expr:
            return None
        n = to_int(expr)
        if n is not None:
            return n
        while expr.startswith('(') and balanced(expr, 0, '(', ')') == len(expr):
            expr = expr[1:-1].strip()
        n = to_int(expr)
        if n is not None:
            return n
        expr = re.sub(r'^\(\s*(?:uint\d+_t|int|unsigned|byte|char)\s*\)\s*', '', expr)
        name = re.sub(r'^(?:\w+::)+', '', expr)
        if re.fullmatch(r'[A-Za-z_]\w*', name) and name in self.consts:
            return self.value(self.consts[name], depth + 1)
        m = re.fullmatch(r'(.+?)\s*(<<|\||>>)\s*(.+)', expr)
        if m:
            a, b = self.value(m.group(1), depth + 1), self.value(m.group(3), depth + 1)
            if a is not None and b is not None:
                return {'<<': a << b, '|': a | b, '>>': a >> b}[m.group(2)]
        m = re.fullmatch(r'_?BV\s*\((.+)\)|bit\s*\((.+)\)', expr)
        if m:
            b = self.value(m.group(1) or m.group(2), depth + 1)
            return None if b is None else 1 << b
        m = re.fullmatch(r'~\s*(.+)', expr)
        if m:
            a = self.value(m.group(1), depth + 1)
            return None if a is None else (~a) & 0xFF
        return None

    def show(self, expr: str) -> str:
        expr = re.sub(r'\s+', ' ', expr.strip())
        v = self.value(expr)
        if v is None:
            return expr
        if to_int(expr) is not None:
            return f'0x{v:02X}'
        return f'{expr} (0x{v:02X})'


# A call whose name says it takes a register address, so a literal argument
# of it is one (readReg(0x83)); readData(8) and readBytes(buf, 4) are not.
TAKES_REGISTER = re.compile(r'reg|Reg|REG|Rd[A-Z]|read8|read16|read24|read32|readByte|I2CRead|i2c_?read|RegRead|_rd', re.I)
NOT_A_REGISTER_NAME = re.compile(r'^(ADDRBIT|AD8_|ADDRESSED_OP|i2c_dev|spi_dev|NULL|nullptr|true|false|HIGH|LOW)')


def register_arg(lib: Library, call: str, args: list):
    """The argument of a read call that names a register: a constant that
    resolves, preferring names that say REG/STATUS/CTRL, else a literal when
    the call takes a register."""
    best = None
    for a in args:
        a = a.strip()
        if not a or a.startswith('&') or re.search(r'\b(buf|buffer|data|len|size|sizeof)\b', a, re.I):
            continue
        name = re.sub(r'^(?:\w+::)+', '', a)
        if NOT_A_REGISTER_NAME.match(name):
            continue
        if re.fullmatch(r'[A-Za-z_]\w*', name):
            if name[0].isupper() and (lib.value(name) is not None or name.isupper()):
                score = 3 if re.search(r'REG|STAT|CTRL|CONF|CMD|MODE|PWR|RESET|CFG|INT', name, re.I) else 2
            elif name[0].islower() and TAKES_REGISTER.search(call):
                score = 0       # a variable: shown by name, never resolved
            else:
                continue
            if best is None or score > best[0]:
                best = (score, a)
        elif TAKES_REGISTER.search(call) and (to_int(a) is not None or lib.value(a) is not None):
            if best is None or best[0] < 1:
                best = (1, a)
    return None if best is None else best[1]


def functions_of(lib: Library):
    """name -> body of every function the library defines, for one level of
    inlining (a loop on getStatus() waits on what getStatus() reads)."""
    if hasattr(lib, '_functions'):
        return lib._functions
    out = {}
    for text in lib.files.values():
        for m in re.finditer(r'\b([A-Za-z_]\w*)\s*\([^;{}()]*(?:\([^()]*\)[^;{}()]*)*\)\s*(?:const\s*)?\{', text):
            name = m.group(1)
            if name in ('if', 'while', 'for', 'switch', 'return', 'catch'):
                continue
            end = balanced(text, m.end() - 1, '{', '}')
            if end and end - m.end() < 4000:
                out.setdefault(name, text[m.end() - 1:end])
    lib._functions = out
    return out


def direct_reads(lib: Library, text: str):
    """(call, register) for each register read call in text."""
    found = []
    for m in READ_CALL.finditer(text):
        name = m.group('name')
        short = re.split(r'\.|->|::', name)[-1]
        if NOT_A_REGISTER_READ.match(name) or NOT_A_REGISTER_READ.match(short) or not READISH.search(short):
            continue
        end = balanced(text, m.end() - 1, '(', ')')
        if end is None:
            continue
        reg = register_arg(lib, short, split_args(text[m.end():end - 1]))
        if reg is not None:
            found.append((name, reg))
    return found


# A chip that answers a bare read with its status byte (AHT10/20, DHT20,
# SHT3x): requestFrom() then read(), or Adafruit_I2CDevice::read(&b, 1).
BARE_READ = re.compile(r'requestFrom\s*\(|(?:->|\.)read\s*\(\s*&?\w+\s*,\s*1\s*\)')
STATUS_BYTE = '(status byte, no pointer)'


def reads_in(lib: Library, text: str, depth: int = 2):
    """(call, register) for each register read in text, looking up to two
    levels into the library's own functions that take no register
    (isMeasuring() -> readStatus() -> the status byte)."""
    found = direct_reads(lib, text)
    if depth == 0:
        return found
    seen = {c for c, _r in found}
    for m in READ_CALL.finditer(text):
        name = m.group('name')
        # Only the library's own functions: Wire.available() is not the
        # available() a radio driver defines.
        if '.' in name or ('->' in name and not name.startswith('this->')):
            continue
        short = re.split(r'->|::', name)[-1]
        if name in seen or NOT_A_REGISTER_READ.match(short) or short in ('delay', 'yield', 'if', 'while'):
            continue
        body = functions_of(lib).get(short)
        if not body or 'digitalRead' in body:
            continue
        inner = reads_in(lib, body, depth - 1)
        if inner:
            found.extend((f'{short}() -> {call}', reg) for call, reg in inner)
        elif BARE_READ.search(body):
            found.append((f'{short}()', STATUS_BYTE))
    return found


def clean_mask(token: str) -> str:
    token = token.strip()
    while token.count(')') > token.count('('):
        token = token[:token.rindex(')')].strip()
    while token.count('(') > token.count(')'):
        token = token[token.index('(') + 1:].strip()
    return token


def masks_in(lib: Library, cond: str):
    masks = []
    for m in re.finditer(r'(?<![&])&(?![&=])\s*(\(?\s*~?\s*[\w:]+(?:\s*(?:<<|\|)\s*[\w:]+)*\s*\)?|\([^()]*\))', cond):
        masks.append(clean_mask(m.group(1)))
    return masks


def polarity(cond: str, needle: str) -> str:
    """What the loop waits for, read from the clause of the condition that
    holds the read: `clear` (it loops while the bits are set), `set`, or
    `value` (a read-back compared with something else)."""
    clauses = re.split(r'&&|\|\|', cond)
    clause = next((c for c in clauses if needle in c), cond)
    c = re.sub(r'\s+', '', clause)
    while c.startswith('(') and balanced(c, 0, '(', ')') == len(c):
        c = c[1:-1]
    if re.search(r'^!|==0(?:x0+)?[uUlL]*\)?$|^\(?0(?:x0+)?==|==false', c):
        return 'set'
    if re.search(r'!=0(?:x0+)?[uUlL]*\)?$|==1\)?$|==true', c):
        return 'clear'
    if re.search(r'[!=]=|[<>]', c.replace('<<', '').replace('>>', '')):
        return 'value'
    return 'clear'


class Hit(dict):
    pass


def lint_library(lib: Library):
    hits = []
    for rel, text in sorted(lib.files.items()):
        if 'while' not in text:
            continue
        # Adafruit_BusIO registers and the bit fields over them, in this file
        # and, for members, in the library's headers.
        registers = {}
        bits = {}
        for src in [text] + [t for r, t in lib.files.items() if r != rel and r.endswith(('.h', '.hpp'))]:
            for m in re.finditer(r'Adafruit_BusIO_Register\s+(\w+)\s*(?:=\s*Adafruit_BusIO_Register\s*)?\(', src):
                end = balanced(src, m.end() - 1, '(', ')')
                if end:
                    args = [a for a in split_args(src[m.end():end - 1])[1:]
                            if not NOT_A_REGISTER_NAME.match(a.strip())]
                    if args:
                        registers.setdefault(m.group(1), args[0])
            for m in re.finditer(r'Adafruit_BusIO_RegisterBits\s+(\w+)\s*(?:=\s*Adafruit_BusIO_RegisterBits\s*)?\(', src):
                end = balanced(src, m.end() - 1, '(', ')')
                if end:
                    args = split_args(src[m.end():end - 1])
                    if len(args) >= 3:
                        reg_obj = args[0].lstrip('&').strip()
                        w, sh = lib.value(args[1]), lib.value(args[2])
                        mask = None if w is None or sh is None else ((1 << w) - 1) << sh
                        pos = m.start() if src is text else -1
                        bits.setdefault(m.group(1), []).append((pos, reg_obj, mask))

        for m in re.finditer(r'\bwhile\s*\(', text):
            cond_end = balanced(text, m.end() - 1, '(', ')')
            if cond_end is None:
                continue
            cond = text[m.end():cond_end - 1]
            line = text.count('\n', 0, m.start()) + 1
            kind = 'while'
            body = ''
            k = m.start() - 1
            while k >= 0 and text[k].isspace():
                k -= 1
            if k >= 0 and text[k] == '}':
                # do { ... } while (...);  -- find the matching "do {".
                depth, j = 0, k
                while j >= 0:
                    if text[j] == '}':
                        depth += 1
                    elif text[j] == '{':
                        depth -= 1
                        if depth == 0:
                            break
                    j -= 1
                if j >= 0 and re.search(r'\bdo\s*$', text[max(0, j - 16):j]):
                    kind = 'do-while'
                    body = text[j:k + 1]
            if kind == 'while':
                after = text[cond_end:cond_end + 4000].lstrip()
                if after.startswith('{'):
                    start = text.index('{', cond_end)
                    end = balanced(text, start, '{', '}')
                    body = text[start:end] if end else ''
                elif not after.startswith(';'):
                    body = after.split(';', 1)[0]

            found = []   # (loop, call, register, mask, needle)
            # Adafruit_BusIO: a bit field read in the condition.
            for name, entries in bits.items():
                if re.search(rf'\b{name}\s*\.\s*read\s*\(', cond):
                    prior = [e for e in entries if e[0] < m.start()] or entries
                    _pos, reg_obj, mask = prior[-1]
                    reg = registers.get(reg_obj, reg_obj)
                    found.append(('RegisterBits', f'{name}.read()', reg,
                                  None if mask is None else f'0x{mask:02X}', name))
            # A register object read in the condition.
            for name, reg in registers.items():
                if re.search(rf'\b{name}\s*\.\s*read\s*\(', cond):
                    for mk in masks_in(lib, cond) or [None]:
                        found.append(('Register', f'{name}.read()', reg, mk, name))
            # A read call in the condition, or in a function it calls.
            for call, reg in reads_in(lib, cond):
                # A mask in the function a loop calls (return status & BUSY).
                inner = functions_of(lib).get(call.split('()')[0]) if '()' in call else None
                returned = ' '.join(re.findall(r'\breturn\b([^;]*);', inner)) if inner else ''
                for mk in masks_in(lib, cond) or masks_in(lib, returned) or [None]:
                    found.append((kind, call, reg, mk, call.split('()')[0].split('.')[-1]))
            # while (1) { ... if (X) break; }: the loop waits on X.
            invert = False
            if re.fullmatch(r'\s*(1|true)\s*', cond) and body:
                for bm in re.finditer(r'\bif\s*\(', body):
                    iend = balanced(body, bm.end() - 1, '(', ')')
                    if iend and re.match(r'\s*\{?\s*break\s*;', body[iend:]):
                        cond, invert = body[bm.end():iend - 1], True
                        break
                if not invert:
                    continue
                for call, reg in reads_in(lib, cond):
                    for mk in masks_in(lib, cond) or [None]:
                        found.append(('while(1)', call, reg, mk, call.split('()')[0].split('.')[-1]))
            # A variable in the condition that the loop reads a register into:
            # v = read(REG); read(REG, &v); or one step removed,
            # ready = status & MASK after read(REG, &status).
            if not found and body:
                names = {v for v in re.findall(r'\b([A-Za-z_]\w*)\b', cond) if not v.isupper()}
                for var in names:
                    sources = [(var, None)]
                    for am in re.finditer(rf'\b{var}\s*(?:\|=|=)(?!=)\s*([^;]+);', body):
                        rhs = am.group(1)
                        for call, reg in reads_in(lib, rhs):
                            for mk in masks_in(lib, cond) or masks_in(lib, rhs) or [None]:
                                found.append((kind, call, reg, mk, var))
                        sources += [(o, rhs) for o in set(re.findall(r'\b([A-Za-z_]\w*)\b', rhs)) - {var}]
                    for other, via in sources:
                        for cm in READ_CALL.finditer(body):
                            cend = balanced(body, cm.end() - 1, '(', ')')
                            if not cend:
                                continue
                            cargs = split_args(body[cm.end():cend - 1])
                            plain = [re.sub(r'\([\w ]+\*\)', '', x).replace(' ', '') for x in cargs]
                            if f'&{other}' not in plain:
                                continue
                            short = re.split(r'\.|->|::', cm.group('name'))[-1]
                            reg = register_arg(lib, short, cargs)
                            if reg is not None and READISH.search(short):
                                for mk in (via and masks_in(lib, via)) or masks_in(lib, cond) or [None]:
                                    found.append((kind, cm.group('name'), reg, mk, var))
            if not found:
                continue
            bounded = bool(BOUNDED.search(cond))
            if body and re.search(r'\b(break|return|goto)\b|[Tt]ime[Oo]ut|millis\s*\(', body):
                bounded = True
            seen = set()
            for loop, call, reg, mask, needle in found:
                key = (call, reg, mask)
                if key in seen:
                    continue
                seen.add(key)
                name = re.sub(r'^(?:\w+::)+', '', reg.strip())
                hits.append(Hit(
                    library=lib.name, file=rel, line=line, loop=loop, call=call,
                    register=lib.show(reg) if not name[:1].islower() else name,
                    register_value=lib.value(reg) if not name[:1].islower() else None,
                    mask=None if mask is None else lib.show(mask),
                    mask_value=None if mask is None else lib.value(mask),
                    waits_for=({'set': 'clear', 'clear': 'set'}.get(polarity(cond, needle), 'value')
                               if invert else polarity(cond, needle)),
                    bounded=bounded,
                    condition=re.sub(r'\s+', ' ', cond.strip())[:120]))
    return hits


def libraries(folders):
    for folder in folders:
        folder = os.path.abspath(folder)
        if any(f.endswith(SOURCE_EXT) for f in os.listdir(folder)) or os.path.isdir(os.path.join(folder, 'src')):
            yield Library(os.path.basename(folder), folder)
            continue
        for child in sorted(os.listdir(folder)):
            path = os.path.join(folder, child)
            # .retired holds versions the cache no longer serves.
            if os.path.isdir(path) and not child.startswith('.'):
                yield Library(child, path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('folders', nargs='+')
    out = ap.add_mutually_exclusive_group()
    out.add_argument('--markdown', action='store_true')
    out.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)

    hits = [h for lib in libraries(args.folders) for h in lint_library(lib)]
    if args.json:
        json.dump(hits, sys.stdout, indent=1)
        print()
    elif args.markdown:
        print('| Library | Source | Loop | Register | Mask | Waits for | Gives up |')
        print('|---|---|---|---|---|---|---|')
        for h in hits:
            print(f"| {h['library']} | {h['file']}:{h['line']} | {h['loop']} `{h['call']}` | "
                  f"{h['register']} | {h['mask'] or '-'} | {h['waits_for']} | "
                  f"{'yes' if h['bounded'] else 'NO'} |")
    else:
        for h in hits:
            print(f"{h['library']}\t{h['file']}:{h['line']}\t{h['loop']}\t{h['call']}\t"
                  f"{h['register']}\t{h['mask'] or '-'}\t{h['waits_for']}\t"
                  f"{'bounded' if h['bounded'] else 'FOREVER'}\t{h['condition']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
