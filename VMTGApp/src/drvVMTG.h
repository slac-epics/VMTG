#ifndef DRV_VMTG_H
#define DRV_VMTG_H
/* $Id$ */

/* VMTG Driver public interface */

#ifdef __cplusplus
extern "C" {
#endif

/* Interrupt bit definitions; note that the VME IRQ level is
 * encoded in the upper bits.
 */

#define REG_IRQ_CTRL_LVL_MASK    0xfe00
#define REG_IRQ_CTRL_LVL(l)      (1<<((l)+8))

#define REG_IRQ_CTRL_EI          (1<<0)
#define REG_IRQ_CTRL_TS0I        (1<<1)
#define REG_IRQ_CTRL_DRI         (1<<2)
#define REG_IRQ_CTRL_TMI         (1<<3)
#define REG_IRQ_CTRL_MS0I        (1<<4)
#define REG_IRQ_CTRL_GTI         (1<<5)
#define REG_IRQ_CTRL_C8MI        (1<<6)
#define REG_IRQ_CTRL_RFMI        (1<<7)
#define REG_IRQ_CTRL_GPI         (1<<8)


/* ISR. The routine is passed a bit-set of 'pending' interrupts
 * and it must return a bitmask of the 'handled' interrupt conditions.
 * 'parm' is the parameter passed to the installation routine.
 */
typedef uint16_t (*VMTGIsr)(uint16_t irqs_pending, void *parm);

/* Install an ISR; RETURNS zero on success, nonzero on error */

long
vmtgInstallISR(VMTGIsr isr, void *parm);

/* Disable interrupts at the VMTG board.
 * RETURNS: bitmask to pass to vmtgIrqsEnable().
 * NOTE:    disable/enable pairs can be nested
 *          and only the outermost 'enable' will
 *          effectively re-enable the interrupts.
 */
uint16_t
vmtgIrqsDisable(void);

/*
 * Enable interrupts. 'key' is a bitmask returned
 * previously by vmtgIrqsDisable().
 *
 * The first time 'vmtgIrqsEnable()' is executed
 * the key must be computed by the user. It is
 * composed of a bit-mask of interrupt conditions
 * AND an (encoded) interrupt level.
 *
 * NOTE: This routine enables interrupts only
 *       at the board. The user must ensure
 *       that the corresponding level is enabled
 *       at the VME controller.
 */
void
vmtgIrqsEnable(uint16_t key);

/* Obtain current time-slot
 * RETURNS:  one-based time-slot, i.e., a number [1..6]
 */
int
vmtgGetTS(void);

/* Obtain current time-slot MODULO 3
 * RETURNS:  one-based time-slot MOD 3, i.e., a number [1..3]
 */
int
vmtgGetTSMod3(void);

#ifdef __cplusplus
}
#endif

#endif
