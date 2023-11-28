/* ----------------------------------------------------------------------------
  Copyright (c) 2021, Microsoft Research, Daan Leijen
  This is free software; you can redistribute it and/or modify it 
  under the terms of the MIT License. A copy of the License can be 
  found in the LICENSE file at the root of this distribution.

  ARM64 (aarch64) calling convention. See:
  - <https://en.wikipedia.org/wiki/Calling_convention#ARM_.28A64.29>
  - <http://infocenter.arm.com/help/topic/com.arm.doc.ihi0055c/IHI0055C_beta_aapcs64.pdf>

  Primitives to switch stacks:
 
    typedef uint8_t mp_jmp_buf_t[MP_JMPBUF_SIZE];  // machine word aligned
  
    bool     mp_setjmp ( mp_jmp_buf_t jmp );
    void     mp_longjmp( mp_jmp_buf_t jmp );
    void*    mp_stack_enter(void* stack_base, void* stack_commit_limit, void* stack_limit, mp_jmpbuf_t** return_jmp, 
                            void (*fun)(void* arg, void* trapframe), void* arg);
    
  `mp_stack_enter` enters a fresh stack and runs `fun(arg)`; it also receives 
  a (pointer to a pointer to a) return jmpbuf to which it longjmp's on return.
-----------------------------------------------------------------------------*/


/*
notes: 
- According to the ARM ABI specification, only the bottom 64 bits of the floating 
  point registers need to be preserved (sec. 5.1.2 of aapcs64).
  Todo: do we need to save the full 128 bits on Windows?
- The x18 register is the "platform register" and may be temporary or not. For safety
  we always save it.

jmpbuf layout:
   0: c18  
  16: c19
  32: c20
  48: c21
  64: c22
  80: c23
  96: c24
 112: c25
 128: c26
 144: c27
 160: c28
 176: fp   = c29
 192: lr   = c30
 208: sp   = c31
 224: fpcr
 232: fpsr
 240: d8  (64 bits)
 248: d9
 ...
 296: d15
 304: sizeof jmpbuf
*/

.align 2
.global mp_setjmp
.global mp_longjmp
.global mp_stack_enter

#if defined(__MACH__)
.global _mp_setjmp
.global _mp_longjmp
.global _mp_stack_enter
#endif

.type mp_setjmp,%function
.type mp_longjmp,%function
.type mp_stack_enter,%function
.type abort,%function

/* called with c0: &jmp_buf */
_mp_setjmp:
mp_setjmp:                 
  .cfi_startproc purecap
  stp   c18, c19, [c0], #32
  stp   c20, c21, [c0], #32
  stp   c22, c23, [c0], #32
  stp   c24, c25, [c0], #32
  stp   c26, c27, [c0], #32
  stp   c28, c29, [c0], #32   /* c28 and fp */
  mov   c10, csp              /* sp to c10 */
  stp   c30, c10, [c0], #32   /* lr and sp */
  /* store fp control and status */
  mrs   x10, fpcr
  mrs   x11, fpsr
  stp   x10, x11, [c0], #16    
  /* store float registers */
  stp   d8,  d9,  [c0], #16
  stp   d10, d11, [c0], #16
  stp   d12, d13, [c0], #16
  stp   d14, d15, [c0], #16
  /* always return zero */
  mov   x0, #0
  ret                         /* jump to lr */
  .cfi_endproc


/* called with x0: &jmp_buf */
_mp_longjmp:
mp_longjmp:                  
  .cfi_startproc purecap
  ldp   c18, c19, [c0], #32
  ldp   c20, c21, [c0], #32
  ldp   c22, c23, [c0], #32
  ldp   c24, c25, [c0], #32
  ldp   c26, c27, [c0], #32
  ldp   c28, c29, [c0], #32   /* c28 and fp */
  ldp   c30, c10, [c0], #32   /* lr and sp */
  mov   csp,  c10
  /* load fp control and status */
  ldp   x10, x11, [c0], #16
  msr   fpcr, x10
  msr   fpsr, x11
  /* load float registers */
  ldp   d8,  d9,  [c0], #16
  ldp   d10, d11, [c0], #16
  ldp   d12, d13, [c0], #16
  ldp   d14, d15, [c0], #16
  /* always return 1 */
  mov   x0, #1
  ret                         /* jump to lr */
  .cfi_endproc


/* switch stack 
   c0: stack pointer, 
   c1: stack commit limit    (ignored on unix)
   c2: stack limit           (ignored on unix)
   c3: jmpbuf_t** return point
   c4: function to run
   c5: argument to pass to the function 
*/
/* DWARF unwind info instructions: <http://dwarfstd.org/doc/DWARF5.pdf> 
   Register mapping: <file:///C:/Users/daan/AppData/Local/Temp/IHI0057B_aadwarf64.pdf>
*/
#define DW_def_cfa_expression     0x0F  
#define DW_expression             0x10      
#define DW_val_expression         0x16        
#define DW_OP_deref               0x06        /* dereference the top of the expression stack */
#define DW_OP_breg(r)             (0x70+r)    /* push `register + ofs` on the expression stack */
#define DW_OP_bregx               0x92        /* push `register + ofs` on the expression stack */
#define DW_REG_cfp                227, 1      /* LEB128 encoded 227 */
#define DW_REG_clr                228, 1
#define DW_REG_csp                229, 1
#define DW_REG_ELR_mode           33
#define DW_REG_v0                 64

_mp_stack_enter:
mp_stack_enter:
  .cfi_startproc purecap
  .cfi_signal_frame             /* needed or else gdb does not allow switching frames to a lower address in the backtrace */
  
  stp     cfp, clr, [csp, #-32]! /* link frame (seems needed for libunwind) */
  /* TODO: check these make some kind of sense? */
  .cfi_adjust_cfa_offset 64
  .cfi_rel_offset clr, 48
  .cfi_rel_offset cfp, 32

  mov     cfp, csp
  sub     csp, csp, #32
  str     c3, [csp, #16]          /* save c3 to [sp+16] for unwinding */
  .cfi_remember_state 
  
  /* set the cfa to point to our return jmpbuf_t (instead of into the stack); 
     the previous registers can now be restored (during unwind) using .cfi_offset directives */
  .cfi_escape DW_def_cfa_expression, 5, DW_OP_bregx, DW_REG_csp, 16, DW_OP_deref, DW_OP_deref /* jmpbuf_t* cfa = [[sp,#16]] */  
  .cfi_offset csp, 208          /* sp */
  .cfi_offset c30, 192          /* lr */
  .cfi_offset c29, 176          /* fp */
  .cfi_offset c18,  0
  .cfi_offset c19, 16
  .cfi_offset c20, 32
  .cfi_offset c21, 48
  .cfi_offset c22, 64
  .cfi_offset c23, 80
  .cfi_offset c24, 96
  .cfi_offset c25, 112
  .cfi_offset c26, 128
  .cfi_offset c27, 144
  .cfi_offset c28, 160 

  alignd  csp, c0, #5           /* switch to the new stack (aligned down to 32 bytes) */
  sub     csp, csp, #64         /* sp = sp - 64 */
  stp     c29, c30, [csp, #32]  /* mimic call on new stack: sp[48] = c30, sp[32] = lr */
  str     c3, [csp, #16]        /* save jmpbuf_t** on the stack for (future) unwinding: c3 == [sp+16]  */
  add     c29, csp, #32         /* set our frame pointer to sp[32] */
  
  mov     c0, c5                /* argument to x0 */
  mov     c1, cfp               /* pass unwind frame */
  blr     c4                    /* and call the function */

  /* should never get here */
  #if defined(__MACH__)
  bl      _abort
  #else
  bl      abort
  #endif

  .cfi_restore_state
  ldr     c0, [csp, #16]        /* load jmpbuf_t* and longjmp back */
  ldr     c0, [c0]
  b       mp_longjmp

  .cfi_endproc
