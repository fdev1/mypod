;
; MyPod - dsPIC Based MP3 Player
; Copyright (C) 2014 Fernando Rodriguez (support@fernansoft.com)
;
; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License Version 3 as 
; published by the Free Software Foundation.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this program; if not, write to the Free Software
; Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
;


;
; void truncate(int* samples, int att_ratio, n_samples, dither);
;
; w0 - 16-bit samples buffer 
; w1 - attenuation ratio in Q15 format
; w2 - sample count
;
.text
.global _truncate
_truncate:
	push w9
	;
	; initialize registers
	;	
	mov w0, w9		; pointer for x memory prefetch
	mov w1, w7		; attenuation ratio
	dec w2, w2		; for hardware do...loop
	mov [w9++], w5	; fetch 1st sample
	mov #127, w4	; for signed->unsigned conv
	;mov #32767, w5
	clr b
	;
	; truncate samples
	;
	do w2, _quant16_loop
	mpy w5 * w7, a;, [w9] += 2, w5		; attenuate sample
	mov [w9++], w5
	add a								
	sftac a, #8							; truncate to 8-bits

	clr b
	add b
	sac a, w2
	neg w2, w2
	add w2, b
	;
	; write back as 8 bit unsigned and
	; fetch the next sample
	;	
	add w4, a		; convert to unsigned
_quant16_loop: 
	sac a, [w0++]	; write back to buffer
	;
	; unstack registers and return
	;
	pop w9
	return

;
; void truncate(int* samples, int att_ratio, n_samples, dither);
;
; w0:w1 - 16-bit samples buffer 
; w2 - attenuation ratio in Q15 format
; w3 - sample count
;
.equ USE_PREFETCH, 1
.text
.global _truncate_eds
.ifdef USE_PREFETCH
_truncate_eds:
	push w11
	;push DSRPAG
	push DSWPAG
	;
	; set paging registers
	;
	btss w0, #15
	inc w1, w1
	;mov w1, DSRPAG
	mov w1, DSWPAG
	;
	; initialize registers
	;	
	mov w0, w11		; pointer for y memory prefetch
	mov w2, w7		; attenuation ratio
	dec w3, w3		; for hardware do...loop
	movsac a, [w11] += 2, w5	; fetch 1st sample
	mov #128, w4	; for signed->unsigned conv
	;mov #32767, w5
	clr b
	;
	; truncate samples
	;
	do w3, _quant16_loop_eds
	mpy w5 * w7, a, [w11] += 2, w5		; attenuate sample
	add a
	sftac a, #8							; truncate to 8-bits

	clr b
	add b
	sac a, w3
	neg w3, w3
	add w3, b
	;
	; write back as 8 bit unsigned and
	; fetch the next sample
	;	
	add w4, a		; convert to unsigned
_quant16_loop_eds: 
	sac a, [w0++]	; write back to buffer
	;
	; unstack registers and return
	;
	pop DSWPAG
	;pop DSRPAG
	pop w9
	return
.else
_truncate_eds:
	push w9
	push DSRPAG
	push DSWPAG
	;
	; set paging registers
	;
	btss w0, #15
	inc w1, w1
	mov w1, DSRPAG
	mov w1, DSWPAG
	;
	; initialize registers
	;	
	mov w0, w9		; pointer for y memory prefetch
	mov w2, w7		; attenuation ratio
	dec w3, w3		; for hardware do...loop
	mov [w9++], w5	; fetch 1st sample
	mov #128, w4	; for signed->unsigned conv
	;mov #32767, w5
	clr b
	;
	; truncate samples
	;
	do w3, _quant16_loop_eds
	mpy w5 * w7, a;, [w9] += 2, w5		; attenuate sample
	add a
	sftac a, #7							; truncate to 8-bits

	clr b
	add b
	sac a, w3
	neg w3, w3
	add w3, b
	;
	; write back as 8 bit unsigned and
	; fetch the next sample
	;	
	add w4, a		; convert to unsigned
	mov [w9++], w5
_quant16_loop_eds: 
	sac a, [w0++]	; write back to buffer
	;
	; unstack registers and return
	;
	pop DSWPAG
	pop DSRPAG
	pop w9
	return
.endif
