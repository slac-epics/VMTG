#ifndef VME64_X_H
#define VME64_X_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Try to be alias rule-safe */

/* This can be a VME address or as seen by the CPU! */
#if 0
/* arrgh; uintptr_t is ulonglong on older rtems */
#define myPRIxPTR PRIx32
typedef uint32_t VME64_Addr;
#else
#define myPRIxPTR PRIxPTR
/* arrgh; uintptr_t is ulonglong on older rtems */
typedef uintptr_t VME64_Addr;
#endif

#define PRIxVME64A myPRIxPTR

#define VME64_CR_SPACING 0x80000

typedef volatile union {
	uint8_t	 c;
	uint16_t s;
	uint32_t l;
	uint64_t q;
} VME64_Reg, *VME64_Reg_p;

typedef union {
	VME64_Addr p;
	VME64_Reg *r;
} VME64_RegAddr;

static __inline__
void vme64_membarrier()
{
#ifdef __PPC__
	asm volatile("eieio");
#endif
}

/*
 * Primitives to read/write 8/16/32-bit registers;
 * offsets are in bytes; addresses are as seen from
 * the  CPU.
 */

static __inline__
uint8_t vme64_in08(VME64_Addr base, uint32_t offset)
{
uint8_t       rval;
VME64_RegAddr a;

	/* this stuff gets optimized away */
	a.p  = base + offset;
	rval = a.r->c;
	vme64_membarrier();
	return rval;
}

static __inline__
uint16_t vme64_in16(VME64_Addr base, uint32_t offset)
{
uint16_t      rval;
VME64_RegAddr a;

	/* this stuff gets optimized away */
	a.p  = base + offset;
	rval = a.r->s;
	vme64_membarrier();
	return rval;
}

static __inline__
uint32_t vme64_in32(VME64_Addr base, uint32_t offset)
{
uint32_t      rval;
VME64_RegAddr a;

	/* this stuff gets optimized away */
	a.p  = base + offset;
	rval = a.r->l;
	vme64_membarrier();
	return rval;
}

static __inline__
void vme64_out08(VME64_Addr base, uint32_t offset, uint8_t val)
{
VME64_RegAddr a;

	/* this stuff gets optimized away */
	a.p    = base + offset;
	a.r->c = val;
	vme64_membarrier();
}

static __inline__
void vme64_out16(VME64_Addr base, uint32_t offset, uint16_t val)
{
VME64_RegAddr a;

	/* this stuff gets optimized away */
	a.p    = base + offset;
	a.r->s = val;
	vme64_membarrier();
}

static __inline__
void vme64_out32(VME64_Addr base, uint32_t offset, uint32_t val)
{
VME64_RegAddr a;

	/* this stuff gets optimized away */
	a.p    = base + offset;
	a.r->l = val;
	vme64_membarrier();
}

/*
 * (Some) VME64(x) CR/CSR offsets and values
 */

#define VME64_CR_OFF_CSUM	0x03
#define VME64_CR_OFF_LEN2	0x07
#define VME64_CR_OFF_LEN1	0x0b
#define VME64_CR_OFF_LEN0	0x0f

#define VME64_CR_OFF_C_ID   0x1f
#define VME64_CR_OFF_R_ID   0x23

#define VME64_CR_OFF_CRWD   0x13
#define VME64_CR_OFF_CSRWD  0x17

#define VME64_CR_OFF_MANID  0x27
#define VME64_CR_OFF_BRDID  0x33
#define VME64_CR_OFF_REVID  0x43

#define VME64_CR_CRWD_D8_O		0x81
#define VME64_CR_CRWD_D8_EO		0x82
#define VME64_CR_CRWD_D16		0x83
#define VME64_CR_CRWD_D32		0x84

#define VME64_CR_OFF_BEG_SN		0xcb
#define VME64_CR_OFF_END_SN		0xd7

#define VME64_CR_OFF_DAWPR(i) 	(0x100+1*4*(i)+3)
#define VME64_CR_OFF_AMCAP(i) 	(0x120+8*4*(i)+3)
#define VME64_CR_OFF_ADEM(i) 	(0x620+4*4*(i)+3)

#define VME64_ADEM_EFM			(1<<0)
#define VME64_ADEM_EFD			(1<<1)
#define VME64_ADEM_DFS			(1<<2)
#define VME64_ADEM_FAF			(1<<3)

#define VME64_CSR_OFF_ADER(i)	(0x7ff60+4*4*(i)+3)

#define VME64_CSR_OFF_BCLR		0x7fff7
#define VME64_CSR_OFF_BSET		0x7fffb

#define VME64_CSR_BIT_RESET		(1<<7)
#define VME64_CSR_BIT_SYSFAIL	(1<<6)
#define VME64_CSR_BIT_MODFAIL	(1<<5)
#define VME64_CSR_BIT_MODENBL	(1<<4)
#define VME64_CSR_BIT_BERR		(1<<3)
#define VME64_CSR_BIT_CRAM		(1<<2)

/* read a 24-bit register composed of 3 bytes spaced by 4 */
static __inline__
uint32_t __vme64CSRegRead24(VME64_Addr base, uint32_t off)
{
int i;
uint32_t rval;
	for ( i=3, rval=0; i>0; i--, off+=4 )
		rval = (rval<<8) | vme64_in08(base, off);
	return rval;
}

/* non-inline version */
uint32_t vme64CSRegRead24(VME64_Addr base, uint32_t off);


/* read a 32-bit register composed of 4 bytes spaced by 4 */
static __inline__
uint32_t __vme64CSRegRead32(VME64_Addr base, uint32_t off)
{
int i;
uint32_t rval;
	for ( i=4, rval=0; i>0; i--, off+=4 )
		rval = (rval<<8) | vme64_in08(base, off);
	return rval;
}

/* non-inline version */
uint32_t vme64CSRegRead32(VME64_Addr base, uint32_t off);

/* read a 64-bit register composed of 8 bytes spaced by 4 */
static __inline__
uint64_t __vme64CSRegRead64(VME64_Addr base, uint32_t off)
{
int i;
uint64_t rval;
	for ( i=8, rval=0; i>0; i--, off+=4 )
		rval = (rval<<8) | vme64_in08(base, off);
	return rval;
}

/* non-inline version */
uint64_t vme64CSRegRead64(VME64_Addr base, uint32_t off);

/* write a 32-bit register composed of 4 bytes spaced by 4 */
static inline
void __vme64CSRegWrite32(VME64_Addr base, uint32_t off, uint32_t val)
{
int i;
	for ( i=4*3; i>=0; i-=4, val>>=8 )
		vme64_out08(base, off+i, val);
}

/* non-inline version */
void vme64CSRegWrite32(VME64_Addr base, uint32_t off, uint32_t val);

/*
 * Read ADER instance # 'idx'
 *
 * NOTE: 'base' is address in CSR space as seen from the CPU
 *       (mapped through VME and PCI bridge).
 */
uint32_t
vme64ReadADER(VME64_Addr base, uint32_t idx);

/*
 * Write ADER instance # 'idx'
 *
 * NOTE: 'base' is address in CSR space as seen from the CPU
 *       (mapped through VME and PCI bridge).
 */
void
vme64WriteADER(VME64_Addr base, uint32_t idx, uint32_t val);

/*
 * Read ADEM instance # 'idx'
 *
 * NOTE: 'base' is address in CSR space as seen from the CPU
 *       (mapped through VME and PCI bridge).
 */
uint32_t
vme64ReadADEM(VME64_Addr base, uint32_t idx);

/*
 * Read AMCAP instance # 'idx'
 *
 * NOTE: 'base' is address in CSR space as seen from the CPU
 *       (mapped through VME and PCI bridge).
 */
uint64_t
vme64ReadAMCAP(VME64_Addr base, uint32_t idx);

/*
 * Program ADER instance # 'idx' for VME address and modifier 'addr' / 'am'
 *
 * NOTE: 'base' is address in CSR space as seen from the CPU
 *       (mapped through VME and PCI bridge).
 *       
 *       'addr', however, is a VME address.
 */
int
vme64SetupADER(VME64_Addr base, uint32_t idx, uint64_t addr, uint32_t am);

/*
 * Verify CR Checksum
 *
 * RETURNS: zero on success, nonzero on failure.
 *
 * NOTE:    'base' is address in CSR space as seen from the CPU
 *          (mapped through VME and PCI bridge).
 *
 *          if 'quiet' is nonzero then no error messages are printed
 *          to stderr. 
 *
 */
uint32_t
vme64CRChecksum(VME64_Addr base, int quiet);

/* Read serial-number info;
 * returns S/N on success or
 * 0xffffffff on failure
 * (S/N longer than 4 bytes or not
 * properly formatted).
 *
 * The standard says S/N info should be
 * on every 4th byte accessible via D08[O].
 *
 * This routine also accepts info that is
 * layed out as up to 4 consecutive bytes
 * but rejects other formats.
 *
 */
uint32_t
vme64ReadSN(VME64_Addr base);

/*
 * Scan geographical address space starting at slot 'start_slot'
 * for a board with manufacturer/board-ids 'man_id'/'brd_id'.
 *
 * RETURNS: Slot number where board was found or '-1' if no board
 *          was found.
 *
 * NOTE:    The 'base' address must be the address of VME CR space
 *          as seen from the CPU.
 */
int
vme64FindBoard(VME64_Addr base, int start_slot, uint32_t man_id, uint32_t brd_id);

#ifdef __cplusplus
};
#endif

#endif
