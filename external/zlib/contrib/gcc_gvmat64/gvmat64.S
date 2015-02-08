


#define BEFORE_JMP .att_syntax
#define AFTER_JMP .intel_syntax noprefix

#ifndef NO_UNDERLINE
#	define	match_init	_match_init
#	define	longest_match	_longest_match
#endif

.intel_syntax noprefix

.globl	match_init, longest_match
.text
longest_match:



#define LocalVarsSize 96

#define chainlenwmask     (rsp + 8 - LocalVarsSize)
#define nicematch         (rsp + 16 - LocalVarsSize)

#define save_rdi        (rsp + 24 - LocalVarsSize)
#define save_rsi        (rsp + 32 - LocalVarsSize)
#define save_rbx        (rsp + 40 - LocalVarsSize)
#define save_rbp        (rsp + 48 - LocalVarsSize)
#define save_r12        (rsp + 56 - LocalVarsSize)
#define save_r13        (rsp + 64 - LocalVarsSize)
#define save_r14        (rsp + 72 - LocalVarsSize)
#define save_r15        (rsp + 80 - LocalVarsSize)



#define    MAX_MATCH              258
#define    MIN_MATCH              3
#define    MIN_LOOKAHEAD          (MAX_MATCH+MIN_MATCH+1)





#define dsWSize          68
#define dsWMask          76
#define dsWindow         80
#define dsPrev           96
#define dsMatchLen       144
#define dsPrevMatch      148
#define dsStrStart       156
#define dsMatchStart     160
#define dsLookahead      164
#define dsPrevLen        168
#define dsMaxChainLen    172
#define dsGoodMatch      188
#define dsNiceMatch      192

#define window_size      [ rcx + dsWSize]
#define WMask            [ rcx + dsWMask]
#define window_ad        [ rcx + dsWindow]
#define prev_ad          [ rcx + dsPrev]
#define strstart         [ rcx + dsStrStart]
#define match_start      [ rcx + dsMatchStart]
#define Lookahead        [ rcx + dsLookahead] //; 0ffffffffh on infozip
#define prev_length      [ rcx + dsPrevLen]
#define max_chain_length [ rcx + dsMaxChainLen]
#define good_match       [ rcx + dsGoodMatch]
#define nice_match       [ rcx + dsNiceMatch]

        mov [save_rbx],rbx
        mov [save_rbp],rbp


        mov rcx,rdi

        mov r8d,esi


        mov [save_r12],r12
        mov [save_r13],r13
        mov [save_r14],r14
        mov [save_r15],r15


//;;; uInt wmask = s->w_mask;
//;;; unsigned chain_length = s->max_chain_length;
//;;; if (s->prev_length >= s->good_match) {
//;;;     chain_length >>= 2;
//;;; }


        mov edi, prev_length
        mov esi, good_match
        mov eax, WMask
        mov ebx, max_chain_length
        cmp edi, esi
        jl  LastMatchGood
        shr ebx, 2
LastMatchGood:

//;;; chainlen is decremented once beforehand so that the function can
//;;; use the sign flag instead of the zero flag for the exit test.
//;;; It is then shifted into the high word, to make room for the wmask
//;;; value, which it will always accompany.

        dec ebx
        shl ebx, 16
        or  ebx, eax

//;;; on zlib only
//;;; if ((uInt)nice_match > s->lookahead) nice_match = s->lookahead;



        mov eax, nice_match
        mov [chainlenwmask], ebx
        mov r10d, Lookahead
        cmp r10d, eax
        cmovnl r10d, eax
        mov [nicematch],r10d



//;;; register Bytef *scan = s->window + s->strstart;
        mov r10, window_ad
        mov ebp, strstart
        lea r13, [r10 + rbp]

//;;; Determine how many bytes the scan ptr is off from being
//;;; dword-aligned.

         mov r9,r13
         neg r13
         and r13,3

//;;; IPos limit = s->strstart > (IPos)MAX_DIST(s) ?
//;;;     s->strstart - (IPos)MAX_DIST(s) : NIL;


        mov eax, window_size
        sub eax, MIN_LOOKAHEAD


        xor edi,edi
        sub ebp, eax

        mov r11d, prev_length

        cmovng ebp,edi

//;;; int best_len = s->prev_length;


//;;; Store the sum of s->window + best_len in esi locally, and in esi.

       lea  rsi,[r10+r11]

//;;; register ush scan_start = *(ushf*)scan;
//;;; register ush scan_end   = *(ushf*)(scan+best_len-1);
//;;; Posf *prev = s->prev;

        movzx r12d,word ptr [r9]
        movzx ebx, word ptr [r9 + r11 - 1]

        mov rdi, prev_ad

//;;; Jump into the main loop.

        mov edx, [chainlenwmask]

        cmp bx,word ptr [rsi + r8 - 1]
        jz  LookupLoopIsZero
				
						
						
LookupLoop1:
        and r8d, edx

        movzx   r8d, word ptr [rdi + r8*2]
        cmp r8d, ebp
        jbe LeaveNow
		
		
		
        sub edx, 0x00010000
		BEFORE_JMP
        js  LeaveNow
		AFTER_JMP

LoopEntry1:
        cmp bx,word ptr [rsi + r8 - 1]
		BEFORE_JMP
        jz  LookupLoopIsZero
		AFTER_JMP

LookupLoop2:
        and r8d, edx

        movzx   r8d, word ptr [rdi + r8*2]
        cmp r8d, ebp
		BEFORE_JMP
        jbe LeaveNow
		AFTER_JMP
        sub edx, 0x00010000
		BEFORE_JMP
        js  LeaveNow
		AFTER_JMP

LoopEntry2:
        cmp bx,word ptr [rsi + r8 - 1]
		BEFORE_JMP
        jz  LookupLoopIsZero
		AFTER_JMP

LookupLoop4:
        and r8d, edx

        movzx   r8d, word ptr [rdi + r8*2]
        cmp r8d, ebp
		BEFORE_JMP
        jbe LeaveNow
		AFTER_JMP
        sub edx, 0x00010000
		BEFORE_JMP
        js  LeaveNow
		AFTER_JMP

LoopEntry4:

        cmp bx,word ptr [rsi + r8 - 1]
		BEFORE_JMP
        jnz LookupLoop1
        jmp LookupLoopIsZero
		AFTER_JMP
.balign 16
LookupLoop:
        and r8d, edx

        movzx   r8d, word ptr [rdi + r8*2]
        cmp r8d, ebp
		BEFORE_JMP
        jbe LeaveNow
		AFTER_JMP
        sub edx, 0x00010000
		BEFORE_JMP
        js  LeaveNow
		AFTER_JMP

LoopEntry:

        cmp bx,word ptr [rsi + r8 - 1]
		BEFORE_JMP
        jnz LookupLoop1
		AFTER_JMP
LookupLoopIsZero:
        cmp     r12w, word ptr [r10 + r8]
		BEFORE_JMP
        jnz LookupLoop1
		AFTER_JMP


//;;; Store the current value of chainlen.
        mov [chainlenwmask], edx
        lea rsi,[r8+r10]
        mov rdx, 0xfffffffffffffef8 //; -(MAX_MATCH_8)
        lea rsi, [rsi + r13 + 0x0108] //;MAX_MATCH_8]
        lea rdi, [r9 + r13 + 0x0108] //;MAX_MATCH_8]

        prefetcht1 [rsi+rdx]
        prefetcht1 [rdi+rdx]


LoopCmps:
        mov rax, [rsi + rdx]
        xor rax, [rdi + rdx]
        jnz LeaveLoopCmps

        mov rax, [rsi + rdx + 8]
        xor rax, [rdi + rdx + 8]
        jnz LeaveLoopCmps8


        mov rax, [rsi + rdx + 8+8]
        xor rax, [rdi + rdx + 8+8]
        jnz LeaveLoopCmps16

        add rdx,8+8+8

		BEFORE_JMP
        jnz  LoopCmps
        jmp  LenMaximum
		AFTER_JMP
		
LeaveLoopCmps16: add rdx,8
LeaveLoopCmps8: add rdx,8
LeaveLoopCmps:

        test    eax, 0x0000FFFF
        jnz LenLower

        test eax,0xffffffff

        jnz LenLower32

        add rdx,4
        shr rax,32
        or ax,ax
		BEFORE_JMP
        jnz LenLower
		AFTER_JMP

LenLower32:
        shr eax,16
        add rdx,2
		
LenLower:		
        sub al, 1
        adc rdx, 0
//;;; Calculate the length of the match. If it is longer than MAX_MATCH,
//;;; then automatically accept it as the best possible match and leave.

        lea rax, [rdi + rdx]
        sub rax, r9
        cmp eax, MAX_MATCH
		BEFORE_JMP
        jge LenMaximum
		AFTER_JMP
        cmp eax, r11d
        jg  LongerMatch

        lea rsi,[r10+r11]

        mov rdi, prev_ad
        mov edx, [chainlenwmask]
		BEFORE_JMP
        jmp LookupLoop
		AFTER_JMP
LongerMatch:
        mov r11d, eax
        mov match_start, r8d
        cmp eax, [nicematch]
		BEFORE_JMP
        jge LeaveNow
		AFTER_JMP

        lea rsi,[r10+rax]

        movzx   ebx, word ptr [r9 + rax - 1]
        mov rdi, prev_ad
        mov edx, [chainlenwmask]
		BEFORE_JMP
        jmp LookupLoop
		AFTER_JMP

//;;; Accept the current string, with the maximum possible length.

LenMaximum:
        mov r11d,MAX_MATCH
        mov match_start, r8d

//;;; if ((uInt)best_len <= s->lookahead) return (uInt)best_len;
//;;; return s->lookahead;

LeaveNow:
        mov eax, Lookahead
        cmp r11d, eax
        cmovng eax, r11d



//;;; Restore the stack and return from whence we came.


//        mov rsi,[save_rsi]
//        mov rdi,[save_rdi]
        mov rbx,[save_rbx]
        mov rbp,[save_rbp]
        mov r12,[save_r12]
        mov r13,[save_r13]
        mov r14,[save_r14]
        mov r15,[save_r15]


        ret 0
//; please don't remove this string !
//; Your can freely use gvmat64 in any free or commercial app
//; but it is far better don't remove the string in the binary!
 //   db     0dh,0ah,"asm686 with masm, optimised assembly code from Brian Raiter, written 1998, converted to amd 64 by Gilles Vollant 2005",0dh,0ah,0


match_init:
  ret 0


