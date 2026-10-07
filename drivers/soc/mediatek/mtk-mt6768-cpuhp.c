// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6768 stock firmware powers a CPU down automatically on PSCI CPU_OFF,
 * but leaves its SPMC power request asserted. PSCI CPU_ON sets that request
 * again and waits forever for power-on acknowledgement unless the request
 * has first been cleared with the MediaTek SiP POWER_DOWN_CORE service.
 * Complete this sequence on a surviving CPU, as the vendor kernel does.
 */
#include <linux/arm-smccc.h>
#include <linux/cpuhotplug.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/psci.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>

#include <asm/smp_plat.h>
#include <uapi/linux/psci.h>

#define MTK_SIP_POWER_DOWN_CORE MTK_SIP_SMC_CMD(0x212)

static int mt6768_cpu_dead(unsigned int cpu)
{
	struct arm_smccc_res res;
	u64 mpidr = cpu_logical_map(cpu);

	/* Never cut power to a CPU which has not completed PSCI CPU_OFF. */
	if (WARN_ON_ONCE(psci_ops.affinity_info(mpidr, 0) !=
			 PSCI_0_2_AFFINITY_LEVEL_OFF))
		return 0;

	/* MT6768 encodes its eight cores in MPIDR Aff1, with Aff0 == 0. */
	arm_smccc_smc(MTK_SIP_POWER_DOWN_CORE, MPIDR_AFFINITY_LEVEL(mpidr, 1),
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		pr_warn("mt6768-cpuhp: CPU%u power-down service returned %ld\n",
			cpu, (long)res.a0);

	return 0;
}

static int __init mt6768_cpuhp_init(void)
{
	int ret;

	if (!of_machine_is_compatible("mediatek,mt6768"))
		return 0;
	if (!psci_ops.cpu_off || !psci_ops.affinity_info)
		return -ENODEV;

	/* BP teardown runs on a surviving CPU after CPUHP_TEARDOWN_CPU. */
	ret = cpuhp_setup_state_nocalls(CPUHP_BP_PREPARE_DYN,
				      "soc/mediatek/mt6768:dead", NULL,
				      mt6768_cpu_dead);
	return ret < 0 ? ret : 0;
}
device_initcall(mt6768_cpuhp_init);
