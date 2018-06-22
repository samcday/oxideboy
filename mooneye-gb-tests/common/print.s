; Copyright (C) 2014-2018 Joonas Javanainen <joonas.javanainen@gmail.com>
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
; SOFTWARE.

.macro print_string_literal ARGS string
  ld bc, _print_sl_data\@
  call print_string
  jr _print_sl_out\@
_print_sl_data\@:
  c_string string
_print_sl_out\@:
.endm

.bank 1 slot 1
.section "Runtime-Print" free
  ; Clobbers:
  ;   AF, BC, DE, HL
  print_load_font:
    ld hl, VRAM + $10
    ld de, font
    ld bc, FONT_SIZE
    call memcpy
    ret

  ; Inputs:
  ;   BC string address
  ; Clobbers:
  ;   AF, BC, HL
  print_string:
-   ld a, (bc)
    cp $00
    jr z, +
    ld (hl+), a
    inc bc
    jr -
+   ret

  ; Inputs:
  ;   A value
  ; Clobbers:
  ;   AF, BC, HL
  print_a:
    ld b, a
    swap b
    call print_digit
    swap b
    jr print_digit

  print_newline:
    ld a, l    
    and $1F
    xor $1F
    ld b, $00
    ld c, a
    inc c
    add hl, bc
    ret

  ; Inputs:
  ;   A value
  ; Clobbers:
  ;   AF, BC, HL
  print_digit:
    ld a, $0F
    and b
    cp $0a
    jr c, +
    add $07
+   add $30
    ld (hl+), a
    ret

  ; Inputs:
  ;   DE pointer to reg_dump
  ; Clobbers:
  ;   AF, BC, DE, HL
  print_regs:
    .macro __print_reg_pair ARGS reg_a reg_b
      inc de
      print_string_literal reg_a
      ld a, (de)
      call print_a

      dec de
      print_string_literal reg_b
      ld a, (de)
      call print_a

      inc de
      inc de
      call print_newline
    .endm

    __print_reg_pair "  A: " "  F: "
    __print_reg_pair "  B: " "  C: "
    __print_reg_pair "  D: " "  E: "
    __print_reg_pair "  H: " "  L: "

    ret
.ends

.bank 1 slot 1
.section "Font" free
font:
  ; 8x8 ASCII bitmap font by Darkrose
  ; http://opengameart.org/content/8x8-ascii-bitmap-font-with-c-source
  .incbin "font.bin" fsize FONT_SIZE
.ends
