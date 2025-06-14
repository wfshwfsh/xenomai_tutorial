#ifndef _ASM_X86_FPU_INTERNAL_H
#define _ASM_X86_FPU_INTERNAL_H

#include <linux/compat.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/dovetail.h>

#include <asm/user.h>
#include <asm/fpu/api.h>
#include <asm/fpu/xstate.h>
#include <asm/fpu/xcr.h>
#include <asm/cpufeature.h>
#include <asm/trace/fpu.h>




#ifdef CONFIG_DOVETAIL

static inline void oob_fpu_set_preempt(struct fpu *fpu)
{
       fpu->preempted = 1;
}

static inline void oob_fpu_clear_preempt(struct fpu *fpu)
{
       fpu->preempted = 0;
}

static inline bool oob_fpu_preempted(struct fpu *old_fpu)
{
       return old_fpu->preempted;
}

#else

static inline bool oob_fpu_preempted(struct fpu *old_fpu)
{
       return false;
}

#endif /* !CONFIG_DOVETAIL */

#endif
