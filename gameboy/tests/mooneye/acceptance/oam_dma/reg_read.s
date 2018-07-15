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

; This test checks what happens if you read the DMA register. Reads should
; always simply return the last written value, regardless of the state of the
; OAM DMA transfer or other things.

; Verified results:
;   pass: DMG, MGB, SGB, SGB2, CGB, AGB, AGS
;   fail: -

.include "common.s"

  call disable_lcd_safe

; First, a simple case where we let the transfer finish and then do the read
; Two rounds to ensure the value really changes on each write.
prepare_part1:
  ld hl, $ff80
  ld de, hiram_proc1
  ld bc, hiram_proc1_end - hiram_proc1
  call memcpy

round1:
  ld a, $9f
  call $ff80
  ldh a, (<DMA)
  cp $9f
  jp nz, fail_round1

round2:
  ld a, $42
  call $ff80
  ldh a, (<DMA)
  cp $42
  jp nz, fail_round2

; This time we read the value after writing to it. The started OAM DMA transfer
; shouldn't have any effect on the value of DMA.
prepare_part2:
  ld hl, $ff80
  ld de, hiram_proc2
  ld bc, hiram_proc2_end - hiram_proc2
  call memcpy

round3:
  ld a, $9f
  call $ff80
  cp $9f
  jp nz, fail_round3

round4:
  ld a, $42
  call $ff80
  cp $42
  jp nz, fail_round4

; Finally, we write twice and then let the transfer finish. We should always
; see the latest written value
prepare_part3:
  ld hl, $ff80
  ld de, hiram_proc3
  ld bc, hiram_proc3_end - hiram_proc3
  call memcpy

round5:
  ld a, $90
  call $ff80
  ldh a, (<DMA)
  cp $8f
  jp nz, fail_round5

round6:
  ld a, $40
  call $ff80
  ldh a, (<DMA)
  cp $3f
  jp nz, fail_round6

finish:
  test_ok

fail_round1:
  test_failure_string "Fail: r1"

fail_round2:
  test_failure_string "Fail: r2"

fail_round3:
  test_failure_string "Fail: r3"

fail_round4:
  test_failure_string "Fail: r4"

fail_round5:
  test_failure_string "Fail: r5"

fail_round6:
  test_failure_string "Fail: r6"

hiram_proc1:
  ldh (<DMA), a
  ld b, 40
- dec b
  jr nz, -
  ret
hiram_proc1_end:

hiram_proc2:
  ldh (<DMA), a
  ldh a, (<DMA)
  ld b, 40
- dec b
  jr nz, -
  ret
hiram_proc2_end:

hiram_proc3:
  ldh (<DMA), a
  dec a
  ldh (<DMA), a
  ld b, 40
- dec b
  jr nz, -
  ret
hiram_proc3_end:
