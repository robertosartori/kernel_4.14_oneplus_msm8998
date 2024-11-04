/*
 * drivers/base/power/main.c - Where the driver meets power management.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 *
 * This file is released under the GPLv2
 *
 *
 * The driver model core calls device_pm_add() when a device is registered.
 * This will initialize the embedded device_pm_info object in the device
 * and add it to the list of power-controlled devices. sysfs entries for
 * controlling device power management will also be added.
 *
 * A separate list is used for keeping track of power info, because the power
 * domain dependencies may differ from the ancestral dependencies that the
 * subsystem list maintains.
 */

#include <linux/device.h>
#include <linux/kallsyms.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/pm-trace.h>
#include <linux/pm_wakeirq.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/async.h>
#include <linux/suspend.h>
#include <trace/events/power.h>
#include <linux/cpufreq.h>
#include <linux/cpuidle.h>
#include <linux/timer.h>
#include <linux/wakeup_reason.h>

#include "../base.h"
#include "power.h"

typedef int (*pm_callback_t)(struct device *);

/*
 * The entries in the dpm_list list are in a depth first order, simply
 * because children are guaranteed to be discovered after parents, and
 * are inserted at the back of the list on discovery.
 *
 * Since device_pm_add() may be called with a device lock held,
 * we must never try to acquire a device lock while holding
 * dpm_list_mutex.
 */

LIST_HEAD(dpm_list);
static LIST_HEAD(dpm_prepared_list);
static LIST_HEAD(dpm_suspended_list);
static LIST_HEAD(dpm_late_early_list);
static LIST_HEAD(dpm_noirq_list);

struct suspend_stats suspend_stats;
static DEFINE_MUTEX(dpm_list_mtx);
static pm_message_t pm_transition;

static int async_error;

static const char *pm_verb(int event)
{
	switch (event) {
	case PM_EVENT_SUSPEND:
		return "suspend";
	case PM_EVENT_RESUME:
		return "resume";
	case PM_EVENT_FREEZE:
		return "freeze";
	case PM_EVENT_QUIESCE:
		return "quiesce";
	case PM_EVENT_HIBERNATE:
		return "hibernate";
	case PM_EVENT_THAW:
		return "thaw";
	case PM_EVENT_RESTORE:
		return "restore";
	case PM_EVENT_RECOVER:
		return "recover";
	default:
		return "(unknown PM event)";
	}
}

/**
 * device_pm_sleep_init - Initialize system suspend-related device fields.
 * @dev: Device object being initialized.
 */
void device_pm_sleep_init(struct device *dev)
{
	dev->power.is_prepared = false;
	dev->power.is_suspended = false;
	dev->power.is_noirq_suspended = false;
	dev->power.is_late_suspended = false;
	init_completion(&dev->power.completion);
	complete_all(&dev->power.completion);
	dev->power.wakeup = NULL;
	INIT_LIST_HEAD(&dev->power.entry);
}

/**
 * device_pm_lock - Lock the list of active devices used by the PM core.
 */
void device_pm_lock(void)
{
	mutex_lock(&dpm_list_mtx);
}

/**
 * device_pm_unlock - Unlock the list of active devices used by the PM core.
 */
void device_pm_unlock(void)
{
	mutex_unlock(&dpm_list_mtx);
}

const char *suspend_deny_list[] = {
"cpu0",
"cpu1",
"cpu2",
"cpu3",
"cpu4",
"cpu5",
"cpu6",
"cpu7",
"vtcon0",
"slimbus",
"ac000000.ramoops",
"pmsg0",
"soc",
"soc:smp2p-mpss",
"soc:smp2p-lpass",
"soc:smp2p-slpi",
"10f004.qcom,gdsc",
"16b004.qcom,gdsc",
"175004.qcom,gdsc",
"17d034.qcom,gdsc",
"17d038.qcom,gdsc",
"c8ce024.syscon",
"c8c1024.qcom,gdsc",
"c8c1040.qcom,gdsc",
"c8c1044.qcom,gdsc",
"c8c34a0.qcom,gdsc",
"c8c3664.qcom,gdsc",
"c8c3674.qcom,gdsc",
"c8c36d4.qcom,gdsc",
"c8c2304.qcom,gdsc",
"5066008.syscon",
"5066004.qcom,gdsc",
"5065130.syscon",
"5066090.syscon",
"soc:timer",
"10ac000.restart",
"778000.memory",
"1f40000.syscon",
"soc:hwlock",
"1d00000.syscon",
"17911000.mailbox",
"1d0501c.mailbox",
"soc:qcom,smem",
"soc:qcom,smp2p_sleepstate",
"soc:qcom,qsee_irq",
"soc:qcom,qsee_irq_bridge",
"soc:qcom,rpm-smd",
"soc:rpm-glink",
"soc:rpm-glink.rpmsg_ctrl.0.0",
"soc:rpm-glink.rpm_requests.-1.-1",
"soc:qcom,rpm-smd:qcom,rpmcc",
"800f000.qcom,spmi",
"soc:qcom,sps",
"msm_sps",
"spmi-0",
"spmi0-00",
"c1b0000.serial",
"soc:qcom,glink",
"800f000.qcom,spmi:qcom,pm8998@0:qcom,revid@100",
"171c0000.slim",
"800f000.qcom,spmi:qcom,pm8998@0:qcom,power-on@800",
"17240000.slim",
"17920000.timer",
"soc:ddr-bw-opp-table",
"soc:qcom,cpubw",
"1008000.qcom,cpu-bwmon",
"soc:qcom,mincpubw",
"soc:qcom,memlat-cpu0",
"soc:qcom,memlat-cpu4",
"800f000.qcom,spmi:qcom,pm8998@0:pinctrl@c000",
"soc:qcom,arm-memlat-mon-0",
"800f000.qcom,spmi:qcom,pm8998@0:qcom,coincell@2800",
"soc:qcom,arm-memlat-mon-4",
"800f000.qcom,spmi:qcom,pm8998@0:qcom,pm8998_rtc",
"800f000.qcom,spmi:qcom,pm8998@0:adc@3100",
"soc:arm64-cpu-erp",
"800f000.qcom,spmi:qcom,pm8998@0:clock-controller@5b00",
"spmi0-01",
"800f000.qcom,spmi:qcom,pm8998@1:regulator@2f00",
"c8c0000.vote-clock-controller",
"regulator.1",
"800f000.qcom,spmi:qcom,pm8998@1:regulator@3800",
"0.qcom,rmtfs_sharedmem",
"soc:qcom,msm_gsi",
"soc:qcom,rmnet-ipa",
"regulator.2",
"spmi0-02",
"1e00000.qcom,ipa",
"soc:qcom,ipa_fws@1e08000",
"soc:qcom,chd_silver",
"800f000.qcom,spmi:qcom,pmi8998@2:qcom,revid@100",
"soc:qcom,chd_gold",
"800f000.qcom,spmi:qcom,pmi8998@2:qcom,misc@900",
"soc:qcom,ghd",
"17900000.qcom,msm-gladiator-v2",
"800f000.qcom,spmi:qcom,pmi8998@2:qcom,power-on@800",
"soc:qcom,glink_pkt",
"soc:qcom,msm-adsprpc-mem",
"soc:qcom,msm_fastrpc",
"soc:qcom,spcom",
"soc:qcom,spss_utils",
"1da7000.ufsphy",
"1db0000.ufsice",
"800f000.qcom,spmi:qcom,pmi8998@2:pinctrl@c000",
"1da4000.ufshc",
"c012000.qusb",
"c010000.ssphy",
"soc:usb_audio_qmi_dev",
"800f000.qcom,spmi:qcom,pmi8998@2:qcom,qpnp-qnovo@1500",
"soc:usb_nop_phy",
"soc:qcom,rpm-smd:rpm-regulator-smpa1",
"17300000.qcom,lpass",
"soc:qcom,rpm-smd:rpm-regulator-smpa1:regulator-s1-level",
"soc:qcom,memshare",
"regulator.3",
"soc:qcom,rpm-smd:rpm-regulator-smpa1:regulator-s1-floor-level",
"4080000.qcom,mss",
"regulator.4",
"soc:qcom,rpm-smd:rpm-regulator-smpa1:regulator-s1-level-ao",
"10aa000.tsens",
"regulator.5",
"10ad000.tsens",
"soc:qcom,rpm-smd:rpm-regulator-smpa1:regulator-cx-cdev",
"86600000.qseecom",
"soc:qcom,rpm-smd:rpm-regulator-smpa2",
"86600000.smcinvoke",
"soc:qcom,rpm-smd:rpm-regulator-smpa2:regulator-s2",
"146bf720.tz-log",
"regulator.6",
"soc:qcom,msm_hdcp",
"soc:qcom,rpm-smd:rpm-regulator-smpa3",
"1de0000.qcrypto",
"soc:qcom,rpm-smd:rpm-regulator-smpa3:regulator-s3",
"1de0000.qcedev",
"regulator.7",
"793000.qrng",
"soc:qcom,rpm-smd:rpm-regulator-smpa4",
"soc:qcom,bcl",
"soc:qcom,rpm-smd:rpm-regulator-smpa4:regulator-s4",
"regulator.8",
"5c00000.qcom,ssc",
"soc:qcom,rpm-smd:rpm-regulator-smpa5",
"cce0000.qcom,venus",
"soc:qcom,rpm-smd:rpm-regulator-smpa5:regulator-s5",
"17817000.qcom,wdt",
"800f000.qcom,spmi:qcom,pmi8998@2:qcom,qpnp-smb2",
"regulator.9",
"soc:qcom,rpm-smd:rpm-regulator-smpa7",
"soc:qcom,rpm-smd:rpm-regulator-smpa7:regulator-s7",
"800f000.qcom,spmi:qcom,pmi8998@2:bcl@4200",
"800f000.qcom,spmi:qcom,pmi8998@2:rradc@4500",
"800f000.qcom,spmi:qcom,pmi8998@2:gpio-leds",
"spmi0-03",
"800f000.qcom,spmi:qcom,pmi8998@3:qcom,pwms@b100",
"800f000.qcom,spmi:qcom,pmi8998@3:pwm@b300",
"800f000.qcom,spmi:qcom,pmi8998@3:pwm@b400",
"800f000.qcom,spmi:qcom,pmi8998@3:pwm@b500",
"800f000.qcom,spmi:qcom,pmi8998@3:qcom,leds@d000",
"800f000.qcom,spmi:qcom,pmi8998@3:qcom,leds@d800",
"800f000.qcom,spmi:qcom,pmi8998@3:qcom,leds@d300",
"800f000.qcom,spmi:qcom,pmi8998@3:qcom,haptics@c000",
"spmi0-04",
"800f000.qcom,spmi:qcom,pm8005@4:qcom,revid@100",
"800f000.qcom,spmi:qcom,pm8005@4:qcom,temp-alarm@2400",
"800f000.qcom,spmi:qcom,pm8005@4:pinctrl@c000",
"spmi0-05",
"800f000.qcom,spmi:qcom,pm8005@5:regulator@1400",
"regulator.10",
"regulator.11",
"soc:qcom,rpm-smd:rpm-regulator-smpa8",
"soc:qcom,rpm-smd:rpm-regulator-smpa8:regulator-s8",
"regulator.12",
"soc:qcom,rpm-smd:rpm-regulator-smpa9",
"soc:qcom,rpm-smd:rpm-regulator-smpa9:regulator-s9-level",
"regulator.13",
"soc:qcom,rpm-smd:rpm-regulator-smpa9:regulator-s9-floor-level",
"regulator.14",
"soc:qcom,rpm-smd:rpm-regulator-smpa9:regulator-s9-level-ao",
"regulator.15",
"1d0101c.qcom,spss",
"soc:qcom,rpm-smd:rpm-regulator-smpa9:regulator-mx-cdev",
"soc:qcom,msm-rtb",
"soc:qcom,rpm-smd:rpm-regulator-ldoa1",
"10a3000.qcom,mpm2-sleep-counter",
"soc:qcom,rpm-smd:rpm-regulator-ldoa1:regulator-l1",
"146bf000.qcom,msm-imem",
"soc:cpu-pmu",
"regulator.16",
"soc:cpuss_dump",
"soc:qcom,rpm-smd:rpm-regulator-ldoa2",
"soc:qcom,msm-ssc-sensors",
"soc:qcom,rpm-smd:rpm-regulator-ldoa2:regulator-l2",
"10b3000.dcc",
"regulator.17",
"soc:qcom,rpm-smd:rpm-regulator-ldoa3",
"soc:qcom,rpm-smd:rpm-regulator-ldoa3:regulator-l3",
"regulator.18",
"18800000.qcom,icnss",
"soc:qcom,rpm-smd:rpm-regulator-ldoa4",
"soc:qcom,rpm-smd:rpm-regulator-ldoa4:regulator-l4-level",
"c1e7000.msm_tspp",
"regulator.19",
"soc:qcom,wil6210",
"soc:qcom,rpm-smd:rpm-regulator-ldoa4:regulator-l4-floor-level",
"soc:wcd9xxx-irq",
"soc:qmi-tmd-devices",
"regulator.20",
"soc:qcom,rpm-smd:rpm-regulator-ldoa5",
"soc:qcom,rpm-smd:rpm-regulator-ldoa5:regulator-l5",
"regulator.21",
"soc:qcom,rpm-smd:rpm-regulator-ldoa6",
"soc:qcom,rpm-smd:rpm-regulator-ldoa6:regulator-l6",
"1fcf004.regulator",
"regulator.22",
"regulator.23",
"17812000.qcom,spm",
"soc:qcom,rpm-smd:rpm-regulator-ldoa7",
"soc:qcom,rpm-smd:rpm-regulator-ldoa7:regulator-l7",
"regulator.24",
"17912000.qcom,spm",
"soc:qcom,rpm-smd:rpm-regulator-ldoa7:regulator-l7-pin-ctrl",
"regulator.25",
"soc:qcom,rpm-smd:rpm-regulator-ldoa8",
"soc:qcom,lpm-levels",
"soc:qcom,rpm-smd:rpm-regulator-ldoa8:regulator-l8",
"200000.qcom,rpm-stats",
"regulator.26",
"200000.qcom,rpm-rail-stats",
"soc:qcom,rpm-smd:rpm-regulator-ldoa9",
"200000.qcom,rpm-log",
"soc:qcom,rpm-smd:rpm-regulator-ldoa9:regulator-l9",
"778150.qcom,rpm-master-stats",
"regulator.27",
"soc:qcom,rpm-smd:rpm-regulator-ldoa10",
"soc:qcom,rpm-smd:rpm-regulator-ldoa10:regulator-l10",
"regulator.28",
"soc:qcom,rpm-smd:rpm-regulator-ldoa11",
"soc:qcom,rpm-smd:rpm-regulator-ldoa11:regulator-l11",
"regulator.29",
"soc:qcom,rpm-smd:rpm-regulator-ldoa12",
"soc:qcom,rpm-smd:rpm-regulator-ldoa12:regulator-l12",
"regulator.30",
"soc:qcom,rpm-smd:rpm-regulator-ldoa13",
"soc:qcom,rpm-smd:rpm-regulator-ldoa13:regulator-l13",
"regulator.31",
"soc:qcom,rpm-smd:rpm-regulator-ldoa14",
"soc:qcom,rpm-smd:rpm-regulator-ldoa14:regulator-l14",
"regulator.32",
"soc:qcom,rpm-smd:rpm-regulator-ldoa15",
"soc:qcom,rpm-smd:rpm-regulator-ldoa15:regulator-l15",
"regulator.33",
"soc:qcom,rpm-smd:rpm-regulator-ldoa16",
"soc:qcom,rpm-smd:rpm-regulator-ldoa16:regulator-l16",
"soc:iommu_test_device",
"soc:iommu_coherent_test_device",
"regulator.34",
"soc:qcom,ion",
"soc:qcom,rpm-smd:rpm-regulator-ldoa17",
"8c0000.qcom,msm-cam",
"soc:qcom,rpm-smd:rpm-regulator-ldoa17:regulator-l17",
"ca34000.qcom,csiphy",
"regulator.35",
"ca35000.qcom,csiphy",
"soc:qcom,rpm-smd:rpm-regulator-ldoa17:regulator-l17-pin-ctrl",
"ca36000.qcom,csiphy",
"regulator.36",
"soc:qcom,rpm-smd:rpm-regulator-ldoa18",
"ca30000.qcom,csid",
"soc:qcom,rpm-smd:rpm-regulator-ldoa18:regulator-l18",
"ca30400.qcom,csid",
"regulator.37",
"ca30800.qcom,csid",
"soc:qcom,rpm-smd:rpm-regulator-ldoa19",
"soc:qcom,rpm-smd:rpm-regulator-ldoa19:regulator-l19",
"ca30c00.qcom,csid",
"soc:qcom,cam_smmu",
"regulator.38",
"soc:qcom,rpm-smd:rpm-regulator-ldoa20",
"caa4000.qcom,fd",
"soc:qcom,rpm-smd:rpm-regulator-ldoa20:regulator-l20",
"ca04000.qcom,cpp",
"regulator.39",
"ca31000.qcom,ispif",
"soc:qcom,rpm-smd:rpm-regulator-ldoa21",
"soc:qcom,rpm-smd:rpm-regulator-ldoa21:regulator-l21",
"ca10000.qcom,vfe0",
"regulator.40",
"ca14000.qcom,vfe1",
"soc:qcom,rpm-smd:rpm-regulator-ldoa22",
"soc:qcom,vfe",
"soc:qcom,rpm-smd:rpm-regulator-ldoa22:regulator-l22",
"ca0c000.qcom,cci",
"regulator.41",
"ca1c000.qcom,jpeg",
"soc:qcom,rpm-smd:rpm-regulator-ldoa23",
"soc:qcom,rpm-smd:rpm-regulator-ldoa23:regulator-l23",
"caa0000.qcom,jpeg",
"regulator.42",
"cc00000.qcom,vidc",
"soc:qcom,rpm-smd:rpm-regulator-ldoa24",
"c880000.qcom,vmem",
"soc:qcom,rpm-smd:rpm-regulator-ldoa24:regulator-l24",
"6048000.tmc",
"regulator.43",
"6046000.replicator",
"soc:qcom,rpm-smd:rpm-regulator-ldoa25",
"6047000.tmc",
"soc:qcom,rpm-smd:rpm-regulator-ldoa25:regulator-l25",
"6045000.funnel",
"6041000.funnel",
"regulator.44",
"6042000.funnel",
"soc:qcom,rpm-smd:rpm-regulator-ldoa25:regulator-l25-pin-ctrl",
"7b70000.funnel",
"regulator.45",
"7b60000.funnel",
"soc:qcom,rpm-smd:rpm-regulator-ldoa26",
"6002000.stm",
"soc:qcom,rpm-smd:rpm-regulator-ldoa26:regulator-l26",
"7840000.etm",
"7940000.etm",
"regulator.46",
"7a40000.etm",
"soc:qcom,rpm-smd:rpm-regulator-ldoa27",
"7b40000.etm",
"soc:qcom,rpm-smd:rpm-regulator-ldoa27:regulator-l27-level",
"7c40000.etm",
"7d40000.etm",
"regulator.47",
"7e40000.etm",
"soc:qcom,rpm-smd:rpm-regulator-ldoa27:regulator-l27-floor-level",
"7f40000.etm",
"6010000.cti",
"6011000.cti",
"regulator.48",
"6012000.cti",
"soc:qcom,rpm-smd:rpm-regulator-ldoa28",
"6013000.cti",
"soc:qcom,rpm-smd:rpm-regulator-ldoa28:regulator-l28",
"6014000.cti",
"6015000.cti",
"regulator.49",
"6016000.cti",
"soc:qcom,rpm-smd:rpm-regulator-vsa1",
"6017000.cti",
"soc:qcom,rpm-smd:rpm-regulator-vsa1:regulator-lvs1",
"6018000.cti",
"regulator.50",
"6019000.cti",
"soc:qcom,rpm-smd:rpm-regulator-vsa2",
"601a000.cti",
"soc:qcom,rpm-smd:rpm-regulator-vsa2:regulator-lvs2",
"601b000.cti",
"regulator.51",
"601c000.cti",
"soc:qcom,rpm-smd:rpm-regulator-bobb",
"601d000.cti",
"soc:qcom,rpm-smd:rpm-regulator-bobb:regulator-bob",
"601e000.cti",
"601f000.cti",
"regulator.52",
"7820000.cti",
"soc:qcom,rpm-smd:rpm-regulator-bobb:regulator-bob-pin1",
"7920000.cti",
"regulator.53",
"7a20000.cti",
"soc:qcom,rpm-smd:rpm-regulator-bobb:regulator-bob-pin2",
"7b20000.cti",
"7c20000.cti",
"regulator.54",
"soc:qcom,rpm-smd:rpm-regulator-bobb:regulator-bob-pin3",
"7d20000.cti",
"7e20000.cti",
"regulator.55",
"7f20000.cti",
"soc:rpm-glink.glink_ssr.-1.-1",
"7b80000.cti",
"7bc1000.cti",
"7b91000.cti",
"6005000.funnel",
"6004000.tpda",
"7038000.tpdm",
"7054000.tpdm",
"704c000.tpdm",
"71d0000.tpdm",
"7050000.tpdm",
"7bc2000.tpda",
"7bc0000.tpdm",
"7043000.tpda",
"7042000.tpdm",
"7191000.tpda",
"7190000.tpdm",
"7b92000.tpda",
"7b90000.tpdm",
"7083000.funnel",
"7082000.tpda",
"7080000.tpdm",
"158000.hwevent",
"6001000.csr",
"soc:modem_etm0",
"soc:audio_etm0",
"soc:rpm_etm0",
"7225000.funnel",
"soc:dummy-tpdm-wcss",
"1620000.ad-hoc-bus",
"soc:devfreq_spdm_cpu",
"soc:devfreq_spdm_gov",
"soc:qcom,kgsl-hyp",
"soc:qcom,kgsl-busmon",
"soc:ddr-bw-opp-table-gpu",
"soc:qcom,gpubw",
"5000000.qcom,kgsl-3d0",
"5040000.qcom,kgsl-iommu",
"3400000.pinctrl",
"gpiochip0",
"gpiochip0",
"soc:qcom,msm-pcm",
"soc:qcom,msm-pcm-routing",
"soc:qcom,msm-compr-dsp",
"soc:qcom,msm-pcm-low-latency",
"soc:qcom,msm-ultra-low-latency",
"soc:qcom,msm-pcm-dsp-noirq",
"soc:qcom,msm-transcode-loopback",
"soc:qcom,msm-compress-dsp",
"soc:qcom,msm-stub-codec",
"soc:qcom,msm-dai-fe",
"soc:qcom,msm-pcm-afe",
"soc:qcom,msm-dai-q6-hdmi",
"soc:qcom,msm-dai-q6-dp",
"soc:qcom,msm-pcm-loopback",
"soc:qcom,msm-pcm-loopback-low-latency",
"soc:qcom,msm-pcm-dtmf",
"soc:qcom,msm-dai-mi2s",
"soc:qcom,msm-dai-cdc-dma",
"soc:qcom,msm-lsm-client",
"soc:qcom,msm-dai-q6",
"soc:qcom,msm-pcm-hostless",
"soc:qcom,msm-audio-apr",
"soc:qcom,msm-pri-auxpcm",
"soc:qcom,msm-sec-auxpcm",
"soc:qcom,msm-tert-auxpcm",
"soc:qcom,msm-quat-auxpcm",
"soc:qcom,msm-quin-auxpcm",
"soc:qcom,msm-hdmi-dba-codec-rx",
"soc:qcom,msm-adsp-loader",
"soc:qcom,msm-dai-tdm-pri-rx",
"soc:qcom,msm-dai-tdm-pri-tx",
"soc:qcom,msm-dai-tdm-sec-rx",
"soc:qcom,msm-dai-tdm-sec-tx",
"soc:qcom,msm-dai-tdm-tert-rx",
"soc:qcom,msm-dai-tdm-tert-tx",
"soc:qcom,msm-dai-tdm-quat-rx",
"soc:qcom,msm-dai-tdm-quat-tx",
"soc:qcom,msm-dai-tdm-quin-rx",
"soc:qcom,msm-dai-tdm-quin-tx",
"soc:qcom,msm-dai-q6-spdif-pri-rx",
"soc:qcom,msm-dai-q6-spdif-pri-tx",
"soc:qcom,msm-dai-q6-spdif-sec-rx",
"soc:qcom,msm-dai-q6-spdif-sec-tx",
"soc:qcom,msm-dai-q6-afe-loopback-tx",
"c900000.qcom,mdss_mdp",
"soc:qcom,mdss_dsi@0",
"soc:qcom,mdss_wb_panel",
"soc:qcom,msm_ext_disp",
"c900000.qcom,mdss_rotator",
"c994a00.qcom,mdss_dsi_pll",
"c996a00.qcom,mdss_dsi_pll",
"c144000.qcom,sps-dma",
"c184000.qcom,sps-dma",
"c175000.spi",
"c1b8000.spi",
"c171000.uart",
"soc:msm_cdc_pinctrl@64",
"170f700c.qcom,avtimer",
"soc:qcom,msm-cpe-lsm",
"soc:qcom,msm-cpe-lsm@3",
"soc:qcom,wcd-dsp-mgr",
"soc:qcom,wcd-dsp-glink",
"soc:msm_cdc_pinctrl@67",
"soc:msm_cdc_pinctrl@68",
"soc:audio_ext_clk",
"c900000.qcom,sde_kms",
"c994000.qcom,sde_dsi_ctrl0",
"c996000.qcom,sde_dsi_ctrl1",
"c994400.qcom,mdss_dsi_phy0",
"c996400.qcom,mdss_dsi_phy1",
"c9a0000.qcom,hdmi_tx_8998",
"soc:qcom,wb-display@0",
"soc:qcom,hdmi-display",
"c9a0000.qcom,hdmi-cec",
"soc:gpio_keys",
"soc:virtual_therm@0",
"soc:tri_state_key",
"soc:fingerprint_detect",
"soc:fpc_fpc1020",
"soc:qcom,camera-flash@0",
"psci",
"vendor",
"vendor:bt_wcn3990",
"writeback",
"pwmchip0",
"pwmchip1",
"pwmchip2",
"regulator.56",
"regulator.57",
"regulator.58",
"regulator.59",
"regulator.60",
"regulator.61",
"regulator.62",
"regulator.63",
"regulator.64",
"regulator.65",
"regulator.66",
"regulator.67",
"regulator.68",
"regulator.69",
"vga_arbiter",
"input0",
"thermal_zone0",
"thermal_zone1",
"thermal_zone2",
"thermal_zone3",
"thermal_zone4",
"thermal_zone5",
"thermal_zone6",
"thermal_zone7",
"thermal_zone8",
"thermal_zone9",
"thermal_zone10",
"thermal_zone11",
"thermal_zone12",
"thermal_zone13",
"thermal_zone14",
"thermal_zone15",
"thermal_zone16",
"thermal_zone17",
"thermal_zone18",
"thermal_zone19",
"thermal_zone20",
"thermal_zone21",
"thermal_zone22",
"thermal_zone23",
"thermal_zone24",
"thermal_zone25",
"thermal_zone26",
"thermal_zone27",
"thermal_zone28",
"thermal_zone29",
"thermal_zone30",
"thermal_zone31",
"thermal_zone32",
"thermal_zone33",
"thermal_zone34",
"thermal_zone35",
"thermal_zone36",
"thermal_zone37",
"thermal_zone38",
"thermal_zone39",
"thermal_zone40",
"thermal_zone41",
"thermal_zone42",
"thermal_zone43",
"thermal_zone44",
"thermal_zone45",
"thermal_zone46",
"thermal_zone47",
"thermal_zone48",
"thermal_zone49",
"thermal_zone50",
"thermal_zone51",
"thermal_zone52",
"thermal_zone53",
"thermal_zone54",
"thermal_zone55",
"thermal_zone56",
"thermal_zone57",
"thermal_zone58",
"thermal_zone59",
"thermal_zone60",
"thermal_zone61",
"thermal_zone62",
"thermal_zone63",
"thermal_zone64",
"thermal_zone65",
"edac",
"mc",
"soc:qcom,ion:qcom,ion-heap@25",
"soc:qcom,ion:qcom,ion-heap@22",
"soc:qcom,ion:qcom,ion-heap@26",
"soc:qcom,ion:qcom,ion-heap@27",
"soc:qcom,ion:qcom,ion-heap@19",
"soc:qcom,ion:qcom,ion-heap@13",
"soc:qcom,ion:qcom,ion-heap@10",
"soc:qcom,ion:qcom,ion-heap@9",
"ion",
"extcon0",
"extcon1",
"soc:qcom,msm_ext_disp:qcom,msm-ext-disp-audio-codec-rx",
"lo",
"regulatory.0",
"rfkill",
"100000.qcom,gcc",
"c8c0000.qcom,mmsscc",
"5065000.qcom,early_gpucc",
"179c8000.cprh-ctrl",
"regulator.70",
"179c4000.cprh-ctrl",
"regulator.71",
"5061000.cpr4-ctrl",
"regulator.72",
"c8ce020.qcom,gdsc",
"regulator.73",
"c179000.i2c",
"i2c-5",
"5-0020",
"c17a000.i2c",
"i2c-6",
"6-0028",
"c1b5000.i2c",
"i2c-7",
"7-0008",
"7-0055",
"7-0026",
"c1b7000.i2c",
"i2c-9",
"9-0036",
"5065000.qcom,gpucc",
"179c0000.qcom,cpu-clock-8998",
"179c0000.qcom,cpu-clock-8998:qcom,limits-dcvs@179ce800",
"179c0000.qcom,cpu-clock-8998:qcom,limits-dcvs@179cc800",
"5066094.qcom,gdsc",
"regulator.74",
"soc:qcom,msm-cpufreq",
"fab-a1noc",
"fab-a2noc",
"fab-bimc",
"fab-cnoc",
"fab-cr_virt",
"fab-gnoc",
"fab-mnoc",
"fab-snoc",
"fab-mnoc-ahb",
"mas-pcie-0",
"mas-usb3",
"mas-ufs",
"mas-blsp-2",
"mas-cnoc-a2noc",
"mas-ipa",
"mas-sdcc-2",
"mas-sdcc-4",
"mas-tsif",
"mas-blsp-1",
"mas-cr-virt-a2noc",
"mas-gnoc-bimc",
"mas-oxili",
"mas-mnoc-bimc",
"mas-snoc-bimc",
"mas-snoc-cnoc",
"mas-qdss-dap",
"mas-crypto-c0",
"mas-apps-proc",
"mas-cnoc-mnoc-mmss-cfg",
"mas-cnoc-mnoc-cfg",
"mas-cpp",
"mas-jpeg",
"mas-mdp-p0",
"mas-mdp-p1",
"mas-rotator",
"mas-venus",
"mas-vfe",
"mas-venus-vmem",
"mas-hmss",
"mas-qdss-bam",
"mas-snoc-cfg",
"mas-bimc-snoc-0",
"mas-bimc-snoc-1",
"mas-a1noc-snoc",
"mas-a2noc-snoc",
"mas-qdss-etr",
"slv-a1noc-snoc",
"slv-a2noc-snoc",
"slv-ebi",
"slv-hmss-l3",
"slv-bimc-snoc-0",
"slv-bimc-snoc-1",
"slv-cnoc-a2noc",
"slv-ssc-cfg",
"slv-mpm",
"slv-pmic-arb",
"slv-tlmm-north",
"slv-pimem-cfg",
"slv-imem-cfg",
"slv-message-ram",
"slv-skl",
"slv-bimc-cfg",
"slv-prng",
"slv-a2noc-cfg",
"slv-ipa",
"slv-tcsr",
"slv-snoc-cfg",
"slv-clk-ctl",
"slv-glm",
"slv-spdm",
"slv-gpuss-cfg",
"slv-cnoc-mnoc-cfg",
"slv-qm-cfg",
"slv-mss-cfg",
"slv-ufs-cfg",
"slv-tlmm-west",
"slv-a1noc-cfg",
"slv-ahb2phy",
"slv-blsp-2",
"slv-pdm",
"slv-usb3-0",
"slv-a1noc-smmu-cfg",
"slv-blsp-1",
"slv-sdcc-2",
"slv-sdcc-4",
"slv-tsif",
"slv-qdss-cfg",
"slv-tlmm-east",
"slv-cnoc-mnoc-mmss-cfg",
"slv-srvc-cnoc",
"slv-cr-virt-a2noc",
"slv-gnoc-bimc",
"slv-camera-cfg",
"slv-camera-throttle-cfg",
"slv-misc-cfg",
"slv-venus-throttle-cfg",
"slv-venus-cfg",
"slv-vmem-cfg",
"slv-mmss-clk-xpu-cfg",
"slv-mmss-clk-cfg",
"slv-display-cfg",
"slv-display-throttle-cfg",
"slv-smmu-cfg",
"slv-mnoc-bimc",
"slv-vmem",
"slv-srvc-mnoc",
"slv-hmss",
"slv-lpass",
"slv-wlan",
"slv-snoc-bimc",
"slv-snoc-cnoc",
"slv-imem",
"slv-pimem",
"slv-qdss-stm",
"slv-pcie-0",
"slv-srvc-snoc",
"null",
"zero",
"full",
"random",
"urandom",
"kmsg",
"tty",
"console",
"tty0",
"vcs",
"vcsa",
"vcs1",
"vcsa1",
"tty1",
"tty2",
"tty3",
"tty4",
"tty5",
"tty6",
"tty7",
"tty8",
"tty9",
"tty10",
"tty11",
"tty12",
"tty13",
"tty14",
"tty15",
"tty16",
"tty17",
"tty18",
"tty19",
"tty20",
"tty21",
"tty22",
"tty23",
"tty24",
"tty25",
"tty26",
"tty27",
"tty28",
"tty29",
"tty30",
"tty31",
"tty32",
"tty33",
"tty34",
"tty35",
"tty36",
"tty37",
"tty38",
"tty39",
"tty40",
"tty41",
"tty42",
"tty43",
"tty44",
"tty45",
"tty46",
"tty47",
"tty48",
"tty49",
"tty50",
"tty51",
"tty52",
"tty53",
"tty54",
"tty55",
"tty56",
"tty57",
"tty58",
"tty59",
"tty60",
"tty61",
"tty62",
"tty63",
};

bool is_device_in_suspend_denied_list(const char *name) {
	int i;
	for (i = 0; i < ARRAY_SIZE(suspend_deny_list); i++) {
		if (strcmp(name, suspend_deny_list[i]) == 0) {
			return true;
		}
	}
	return false;
}

/**
 * device_pm_add - Add a device to the PM core's list of active devices.
 * @dev: Device to add to the list.
 */
void device_pm_add(struct device *dev)
{
	/* Skip PM setup/initialization. */
	if (device_pm_not_required(dev))
		return;

	if (is_device_in_suspend_denied_list(dev_name(dev)))
		return;

	pr_debug("PM: Adding info for %s:%s\n",
		 dev->bus ? dev->bus->name : "No Bus", dev_name(dev));
	device_pm_check_callbacks(dev);
	mutex_lock(&dpm_list_mtx);
	if (dev->parent && dev->parent->power.is_prepared)
		dev_warn(dev, "parent %s should not be sleeping\n",
			dev_name(dev->parent));
	list_add_tail(&dev->power.entry, &dpm_list);
	dev->power.in_dpm_list = true;
	mutex_unlock(&dpm_list_mtx);
}

/**
 * device_pm_remove - Remove a device from the PM core's list of active devices.
 * @dev: Device to be removed from the list.
 */
void device_pm_remove(struct device *dev)
{
	if (device_pm_not_required(dev))
		return;

	pr_debug("PM: Removing info for %s:%s\n",
		 dev->bus ? dev->bus->name : "No Bus", dev_name(dev));
	complete_all(&dev->power.completion);
	mutex_lock(&dpm_list_mtx);
	list_del_init(&dev->power.entry);
	dev->power.in_dpm_list = false;
	mutex_unlock(&dpm_list_mtx);
	device_wakeup_disable(dev);
	pm_runtime_remove(dev);
	device_pm_check_callbacks(dev);
}

/**
 * device_pm_move_before - Move device in the PM core's list of active devices.
 * @deva: Device to move in dpm_list.
 * @devb: Device @deva should come before.
 */
void device_pm_move_before(struct device *deva, struct device *devb)
{
	pr_debug("PM: Moving %s:%s before %s:%s\n",
		 deva->bus ? deva->bus->name : "No Bus", dev_name(deva),
		 devb->bus ? devb->bus->name : "No Bus", dev_name(devb));
	/* Delete deva from dpm_list and reinsert before devb. */
	list_move_tail(&deva->power.entry, &devb->power.entry);
}

/**
 * device_pm_move_after - Move device in the PM core's list of active devices.
 * @deva: Device to move in dpm_list.
 * @devb: Device @deva should come after.
 */
void device_pm_move_after(struct device *deva, struct device *devb)
{
	pr_debug("PM: Moving %s:%s after %s:%s\n",
		 deva->bus ? deva->bus->name : "No Bus", dev_name(deva),
		 devb->bus ? devb->bus->name : "No Bus", dev_name(devb));

	/* Delete deva from dpm_list and reinsert after devb. */
	list_move(&deva->power.entry, &devb->power.entry);
}

/**
 * device_pm_move_last - Move device to end of the PM core's list of devices.
 * @dev: Device to move in dpm_list.
 */
void device_pm_move_last(struct device *dev)
{
	if (is_device_in_suspend_denied_list(dev_name(dev)))
		return;

	pr_debug("PM: Moving %s:%s to end of list\n",
		 dev->bus ? dev->bus->name : "No Bus", dev_name(dev));

	list_move_tail(&dev->power.entry, &dpm_list);
}

static ktime_t initcall_debug_start(struct device *dev)
{
	ktime_t calltime = 0;

	if (pm_print_times_enabled) {
		pr_info("calling  %s+ @ %i, parent: %s\n",
			dev_name(dev), task_pid_nr(current),
			dev->parent ? dev_name(dev->parent) : "none");
		calltime = ktime_get();
	}

	return calltime;
}

static void initcall_debug_report(struct device *dev, ktime_t calltime,
				  int error, pm_message_t state,
				  const char *info)
{
	ktime_t rettime;
	s64 nsecs;

	rettime = ktime_get();
	nsecs = (s64) ktime_to_ns(ktime_sub(rettime, calltime));

	if (pm_print_times_enabled) {
		pr_info("call %s+ returned %d after %Ld usecs\n", dev_name(dev),
			error, (unsigned long long)nsecs >> 10);
	}
}

/**
 * dpm_wait - Wait for a PM operation to complete.
 * @dev: Device to wait for.
 * @async: If unset, wait only if the device's power.async_suspend flag is set.
 */
static void dpm_wait(struct device *dev, bool async)
{
	if (!dev)
		return;

	if (async || (pm_async_enabled && dev->power.async_suspend))
		wait_for_completion(&dev->power.completion);
}

static int dpm_wait_fn(struct device *dev, void *async_ptr)
{
	dpm_wait(dev, *((bool *)async_ptr));
	return 0;
}

static void dpm_wait_for_children(struct device *dev, bool async)
{
       device_for_each_child(dev, &async, dpm_wait_fn);
}

static void dpm_wait_for_suppliers(struct device *dev, bool async)
{
	struct device_link *link;
	int idx;

	idx = device_links_read_lock();

	/*
	 * If the supplier goes away right after we've checked the link to it,
	 * we'll wait for its completion to change the state, but that's fine,
	 * because the only things that will block as a result are the SRCU
	 * callbacks freeing the link objects for the links in the list we're
	 * walking.
	 */
	list_for_each_entry_rcu(link, &dev->links.suppliers, c_node)
		if (READ_ONCE(link->status) != DL_STATE_DORMANT)
			dpm_wait(link->supplier, async);

	device_links_read_unlock(idx);
}

static bool dpm_wait_for_superior(struct device *dev, bool async)
{
	struct device *parent;

	/*
	 * If the device is resumed asynchronously and the parent's callback
	 * deletes both the device and the parent itself, the parent object may
	 * be freed while this function is running, so avoid that by reference
	 * counting the parent once more unless the device has been deleted
	 * already (in which case return right away).
	 */
	mutex_lock(&dpm_list_mtx);

	if (!device_pm_initialized(dev)) {
		mutex_unlock(&dpm_list_mtx);
		return false;
	}

	parent = get_device(dev->parent);

	mutex_unlock(&dpm_list_mtx);

	dpm_wait(parent, async);
	put_device(parent);

	dpm_wait_for_suppliers(dev, async);

	/*
	 * If the parent's callback has deleted the device, attempting to resume
	 * it would be invalid, so avoid doing that then.
	 */
	return device_pm_initialized(dev);
}

static void dpm_wait_for_consumers(struct device *dev, bool async)
{
	struct device_link *link;
	int idx;

	idx = device_links_read_lock();

	/*
	 * The status of a device link can only be changed from "dormant" by a
	 * probe, but that cannot happen during system suspend/resume.  In
	 * theory it can change to "dormant" at that time, but then it is
	 * reasonable to wait for the target device anyway (eg. if it goes
	 * away, it's better to wait for it to go away completely and then
	 * continue instead of trying to continue in parallel with its
	 * unregistration).
	 */
	list_for_each_entry_rcu(link, &dev->links.consumers, s_node)
		if (READ_ONCE(link->status) != DL_STATE_DORMANT)
			dpm_wait(link->consumer, async);

	device_links_read_unlock(idx);
}

static void dpm_wait_for_subordinate(struct device *dev, bool async)
{
	dpm_wait_for_children(dev, async);
	dpm_wait_for_consumers(dev, async);
}

/**
 * pm_op - Return the PM operation appropriate for given PM event.
 * @ops: PM operations to choose from.
 * @state: PM transition of the system being carried out.
 */
static pm_callback_t pm_op(const struct dev_pm_ops *ops, pm_message_t state)
{
	switch (state.event) {
#ifdef CONFIG_SUSPEND
	case PM_EVENT_SUSPEND:
		return ops->suspend;
	case PM_EVENT_RESUME:
		return ops->resume;
#endif /* CONFIG_SUSPEND */
#ifdef CONFIG_HIBERNATE_CALLBACKS
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		return ops->freeze;
	case PM_EVENT_HIBERNATE:
		return ops->poweroff;
	case PM_EVENT_THAW:
	case PM_EVENT_RECOVER:
		return ops->thaw;
		break;
	case PM_EVENT_RESTORE:
		return ops->restore;
#endif /* CONFIG_HIBERNATE_CALLBACKS */
	}

	return NULL;
}

/**
 * pm_late_early_op - Return the PM operation appropriate for given PM event.
 * @ops: PM operations to choose from.
 * @state: PM transition of the system being carried out.
 *
 * Runtime PM is disabled for @dev while this function is being executed.
 */
static pm_callback_t pm_late_early_op(const struct dev_pm_ops *ops,
				      pm_message_t state)
{
	switch (state.event) {
#ifdef CONFIG_SUSPEND
	case PM_EVENT_SUSPEND:
		return ops->suspend_late;
	case PM_EVENT_RESUME:
		return ops->resume_early;
#endif /* CONFIG_SUSPEND */
#ifdef CONFIG_HIBERNATE_CALLBACKS
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		return ops->freeze_late;
	case PM_EVENT_HIBERNATE:
		return ops->poweroff_late;
	case PM_EVENT_THAW:
	case PM_EVENT_RECOVER:
		return ops->thaw_early;
	case PM_EVENT_RESTORE:
		return ops->restore_early;
#endif /* CONFIG_HIBERNATE_CALLBACKS */
	}

	return NULL;
}

/**
 * pm_noirq_op - Return the PM operation appropriate for given PM event.
 * @ops: PM operations to choose from.
 * @state: PM transition of the system being carried out.
 *
 * The driver of @dev will not receive interrupts while this function is being
 * executed.
 */
static pm_callback_t pm_noirq_op(const struct dev_pm_ops *ops, pm_message_t state)
{
	switch (state.event) {
#ifdef CONFIG_SUSPEND
	case PM_EVENT_SUSPEND:
		return ops->suspend_noirq;
	case PM_EVENT_RESUME:
		return ops->resume_noirq;
#endif /* CONFIG_SUSPEND */
#ifdef CONFIG_HIBERNATE_CALLBACKS
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		return ops->freeze_noirq;
	case PM_EVENT_HIBERNATE:
		return ops->poweroff_noirq;
	case PM_EVENT_THAW:
	case PM_EVENT_RECOVER:
		return ops->thaw_noirq;
	case PM_EVENT_RESTORE:
		return ops->restore_noirq;
#endif /* CONFIG_HIBERNATE_CALLBACKS */
	}

	return NULL;
}

static void pm_dev_dbg(struct device *dev, pm_message_t state, const char *info)
{
	dev_dbg(dev, "%s%s%s\n", info, pm_verb(state.event),
		((state.event & PM_EVENT_SLEEP) && device_may_wakeup(dev)) ?
		", may wakeup" : "");
}

static void pm_dev_err(struct device *dev, pm_message_t state, const char *info,
			int error)
{
	printk(KERN_ERR "PM: Device %s failed to %s%s: error %d\n",
		dev_name(dev), pm_verb(state.event), info, error);
}

static void dpm_show_time(ktime_t starttime, pm_message_t state, int error,
			  const char *info)
{
	ktime_t calltime;
	u64 usecs64;
	int usecs;

	calltime = ktime_get();
	usecs64 = ktime_to_ns(ktime_sub(calltime, starttime));
	do_div(usecs64, NSEC_PER_USEC);
	usecs = usecs64;
	if (usecs == 0)
		usecs = 1;

	pm_pr_dbg("%s%s%s of devices %s after %ld.%03ld msecs\n",
		  info ?: "", info ? " " : "", pm_verb(state.event),
		  error ? "aborted" : "complete",
		  usecs / USEC_PER_MSEC, usecs % USEC_PER_MSEC);
}

static int dpm_run_callback(pm_callback_t cb, struct device *dev,
			    pm_message_t state, const char *info)
{
	ktime_t calltime;
	int error;

	if (!cb)
		return 0;

	calltime = initcall_debug_start(dev);

	pm_dev_dbg(dev, state, info);
	trace_device_pm_callback_start(dev, info, state.event);
	error = cb(dev);
	trace_device_pm_callback_end(dev, error);
	suspend_report_result(cb, error);

	initcall_debug_report(dev, calltime, error, state, info);

	return error;
}

#ifdef CONFIG_DPM_WATCHDOG
struct dpm_watchdog {
	struct device		*dev;
	struct task_struct	*tsk;
	struct timer_list	timer;
};

#define DECLARE_DPM_WATCHDOG_ON_STACK(wd) \
	struct dpm_watchdog wd

/**
 * dpm_watchdog_handler - Driver suspend / resume watchdog handler.
 * @data: Watchdog object address.
 *
 * Called when a driver has timed out suspending or resuming.
 * There's not much we can do here to recover so panic() to
 * capture a crash-dump in pstore.
 */
static void dpm_watchdog_handler(unsigned long data)
{
	struct dpm_watchdog *wd = (void *)data;

	dev_emerg(wd->dev, "**** DPM device timeout ****\n");
	show_stack(wd->tsk, NULL);
	panic("%s %s: unrecoverable failure\n",
		dev_driver_string(wd->dev), dev_name(wd->dev));
}

/**
 * dpm_watchdog_set - Enable pm watchdog for given device.
 * @wd: Watchdog. Must be allocated on the stack.
 * @dev: Device to handle.
 */
static void dpm_watchdog_set(struct dpm_watchdog *wd, struct device *dev)
{
	struct timer_list *timer = &wd->timer;

	wd->dev = dev;
	wd->tsk = current;

	init_timer_on_stack(timer);
	/* use same timeout value for both suspend and resume */
	timer->expires = jiffies + HZ * CONFIG_DPM_WATCHDOG_TIMEOUT;
	timer->function = dpm_watchdog_handler;
	timer->data = (unsigned long)wd;
	add_timer(timer);
}

/**
 * dpm_watchdog_clear - Disable suspend/resume watchdog.
 * @wd: Watchdog to disable.
 */
static void dpm_watchdog_clear(struct dpm_watchdog *wd)
{
	struct timer_list *timer = &wd->timer;

	del_timer_sync(timer);
	destroy_timer_on_stack(timer);
}
#else
#define DECLARE_DPM_WATCHDOG_ON_STACK(wd)
#define dpm_watchdog_set(x, y)
#define dpm_watchdog_clear(x)
#endif

/*------------------------- Resume routines -------------------------*/

/**
 * device_resume_noirq - Execute an "early resume" callback for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being resumed asynchronously.
 *
 * The driver of @dev will not receive interrupts while this function is being
 * executed.
 */
static int device_resume_noirq(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;

	TRACE_DEVICE(dev);
	TRACE_RESUME(0);

	if (dev->power.syscore || dev->power.direct_complete)
		goto Out;

	if (!dev->power.is_noirq_suspended)
		goto Out;

	if (!dpm_wait_for_superior(dev, async))
		goto Out;

	if (dev->pm_domain) {
		info = "noirq power domain ";
		callback = pm_noirq_op(&dev->pm_domain->ops, state);
	} else if (dev->type && dev->type->pm) {
		info = "noirq type ";
		callback = pm_noirq_op(dev->type->pm, state);
	} else if (dev->class && dev->class->pm) {
		info = "noirq class ";
		callback = pm_noirq_op(dev->class->pm, state);
	} else if (dev->bus && dev->bus->pm) {
		info = "noirq bus ";
		callback = pm_noirq_op(dev->bus->pm, state);
	}

	if (!callback && dev->driver && dev->driver->pm) {
		info = "noirq driver ";
		callback = pm_noirq_op(dev->driver->pm, state);
	}

	error = dpm_run_callback(callback, dev, state, info);
	dev->power.is_noirq_suspended = false;

 Out:
	complete_all(&dev->power.completion);
	TRACE_RESUME(error);
	return error;
}

static bool is_async(struct device *dev)
{
	return dev->power.async_suspend && pm_async_enabled
		&& !pm_trace_is_enabled();
}

static void async_resume_noirq(void *data, async_cookie_t cookie)
{
	struct device *dev = (struct device *)data;
	int error;

	error = device_resume_noirq(dev, pm_transition, true);
	if (error)
		pm_dev_err(dev, pm_transition, " async", error);

	put_device(dev);
}

void dpm_noirq_resume_devices(pm_message_t state)
{
	struct device *dev;
	ktime_t starttime = ktime_get();

	trace_suspend_resume(TPS("dpm_resume_noirq"), state.event, true);
	mutex_lock(&dpm_list_mtx);
	pm_transition = state;

	/*
	 * Advanced the async threads upfront,
	 * in case the starting of async threads is
	 * delayed by non-async resuming devices.
	 */
	list_for_each_entry(dev, &dpm_noirq_list, power.entry) {
		reinit_completion(&dev->power.completion);
		if (is_async(dev)) {
			get_device(dev);
			async_schedule(async_resume_noirq, dev);
		}
	}

	while (!list_empty(&dpm_noirq_list)) {
		dev = to_device(dpm_noirq_list.next);
		get_device(dev);
		list_move_tail(&dev->power.entry, &dpm_late_early_list);
		mutex_unlock(&dpm_list_mtx);

		if (!is_async(dev)) {
			int error;

			error = device_resume_noirq(dev, state, false);
			if (error) {
				suspend_stats.failed_resume_noirq++;
				dpm_save_failed_step(SUSPEND_RESUME_NOIRQ);
				dpm_save_failed_dev(dev_name(dev));
				pm_dev_err(dev, state, " noirq", error);
			}
		}

		mutex_lock(&dpm_list_mtx);
		put_device(dev);
	}
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	dpm_show_time(starttime, state, 0, "noirq");
	trace_suspend_resume(TPS("dpm_resume_noirq"), state.event, false);
}

void dpm_noirq_end(void)
{
	resume_device_irqs();
	device_wakeup_disarm_wake_irqs();
	cpuidle_resume();
}

/**
 * dpm_resume_noirq - Execute "noirq resume" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 *
 * Invoke the "noirq" resume callbacks for all devices in dpm_noirq_list and
 * allow device drivers' interrupt handlers to be called.
 */
void dpm_resume_noirq(pm_message_t state)
{
	dpm_noirq_resume_devices(state);
	dpm_noirq_end();
}

/**
 * device_resume_early - Execute an "early resume" callback for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being resumed asynchronously.
 *
 * Runtime PM is disabled for @dev while this function is being executed.
 */
static int device_resume_early(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;

	TRACE_DEVICE(dev);
	TRACE_RESUME(0);

	if (dev->power.syscore || dev->power.direct_complete)
		goto Out;

	if (!dev->power.is_late_suspended)
		goto Out;

	if (!dpm_wait_for_superior(dev, async))
		goto Out;

	if (dev->pm_domain) {
		info = "early power domain ";
		callback = pm_late_early_op(&dev->pm_domain->ops, state);
	} else if (dev->type && dev->type->pm) {
		info = "early type ";
		callback = pm_late_early_op(dev->type->pm, state);
	} else if (dev->class && dev->class->pm) {
		info = "early class ";
		callback = pm_late_early_op(dev->class->pm, state);
	} else if (dev->bus && dev->bus->pm) {
		info = "early bus ";
		callback = pm_late_early_op(dev->bus->pm, state);
	}

	if (!callback && dev->driver && dev->driver->pm) {
		info = "early driver ";
		callback = pm_late_early_op(dev->driver->pm, state);
	}

	error = dpm_run_callback(callback, dev, state, info);
	dev->power.is_late_suspended = false;

 Out:
	TRACE_RESUME(error);

	pm_runtime_enable(dev);
	complete_all(&dev->power.completion);
	return error;
}

static void async_resume_early(void *data, async_cookie_t cookie)
{
	struct device *dev = (struct device *)data;
	int error;

	error = device_resume_early(dev, pm_transition, true);
	if (error)
		pm_dev_err(dev, pm_transition, " async", error);

	put_device(dev);
}

/**
 * dpm_resume_early - Execute "early resume" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 */
void dpm_resume_early(pm_message_t state)
{
	struct device *dev;
	ktime_t starttime = ktime_get();

	trace_suspend_resume(TPS("dpm_resume_early"), state.event, true);
	mutex_lock(&dpm_list_mtx);
	pm_transition = state;

	/*
	 * Advanced the async threads upfront,
	 * in case the starting of async threads is
	 * delayed by non-async resuming devices.
	 */
	list_for_each_entry(dev, &dpm_late_early_list, power.entry) {
		reinit_completion(&dev->power.completion);
		if (is_async(dev)) {
			get_device(dev);
			async_schedule(async_resume_early, dev);
		}
	}

	while (!list_empty(&dpm_late_early_list)) {
		dev = to_device(dpm_late_early_list.next);
		get_device(dev);
		list_move_tail(&dev->power.entry, &dpm_suspended_list);
		mutex_unlock(&dpm_list_mtx);

		if (!is_async(dev)) {
			int error;

			error = device_resume_early(dev, state, false);
			if (error) {
				suspend_stats.failed_resume_early++;
				dpm_save_failed_step(SUSPEND_RESUME_EARLY);
				dpm_save_failed_dev(dev_name(dev));
				pm_dev_err(dev, state, " early", error);
			}
		}
		mutex_lock(&dpm_list_mtx);
		put_device(dev);
	}
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	dpm_show_time(starttime, state, 0, "early");
	trace_suspend_resume(TPS("dpm_resume_early"), state.event, false);
}

/**
 * dpm_resume_start - Execute "noirq" and "early" device callbacks.
 * @state: PM transition of the system being carried out.
 */
void dpm_resume_start(pm_message_t state)
{
	dpm_resume_noirq(state);
	dpm_resume_early(state);
}
EXPORT_SYMBOL_GPL(dpm_resume_start);

/**
 * device_resume - Execute "resume" callbacks for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being resumed asynchronously.
 */
static int device_resume(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;
	DECLARE_DPM_WATCHDOG_ON_STACK(wd);

	TRACE_DEVICE(dev);
	TRACE_RESUME(0);

	if (dev->power.syscore)
		goto Complete;

	if (dev->power.direct_complete) {
		/* Match the pm_runtime_disable() in __device_suspend(). */
		pm_runtime_enable(dev);
		goto Complete;
	}

	if (!dpm_wait_for_superior(dev, async))
		goto Complete;

	dpm_watchdog_set(&wd, dev);
	device_lock(dev);

	/*
	 * This is a fib.  But we'll allow new children to be added below
	 * a resumed device, even if the device hasn't been completed yet.
	 */
	dev->power.is_prepared = false;

	if (!dev->power.is_suspended)
		goto Unlock;

	if (dev->pm_domain) {
		info = "power domain ";
		callback = pm_op(&dev->pm_domain->ops, state);
		goto Driver;
	}

	if (dev->type && dev->type->pm) {
		info = "type ";
		callback = pm_op(dev->type->pm, state);
		goto Driver;
	}

	if (dev->class) {
		if (dev->class->pm) {
			info = "class ";
			callback = pm_op(dev->class->pm, state);
			goto Driver;
		} else if (dev->class->resume) {
			info = "legacy class ";
			callback = dev->class->resume;
			goto End;
		}
	}

	if (dev->bus) {
		if (dev->bus->pm) {
			info = "bus ";
			callback = pm_op(dev->bus->pm, state);
		} else if (dev->bus->resume) {
			info = "legacy bus ";
			callback = dev->bus->resume;
			goto End;
		}
	}

 Driver:
	if (!callback && dev->driver && dev->driver->pm) {
		info = "driver ";
		callback = pm_op(dev->driver->pm, state);
	}

 End:
	error = dpm_run_callback(callback, dev, state, info);
	dev->power.is_suspended = false;

 Unlock:
	device_unlock(dev);
	dpm_watchdog_clear(&wd);

 Complete:
	complete_all(&dev->power.completion);

	TRACE_RESUME(error);

	return error;
}

static void async_resume(void *data, async_cookie_t cookie)
{
	struct device *dev = (struct device *)data;
	int error;

	error = device_resume(dev, pm_transition, true);
	if (error)
		pm_dev_err(dev, pm_transition, " async", error);
	put_device(dev);
}

/**
 * dpm_resume - Execute "resume" callbacks for non-sysdev devices.
 * @state: PM transition of the system being carried out.
 *
 * Execute the appropriate "resume" callback for all devices whose status
 * indicates that they are suspended.
 */
void dpm_resume(pm_message_t state)
{
	struct device *dev;
	ktime_t starttime = ktime_get();

	trace_suspend_resume(TPS("dpm_resume"), state.event, true);
	might_sleep();

	mutex_lock(&dpm_list_mtx);
	pm_transition = state;
	async_error = 0;

	list_for_each_entry(dev, &dpm_suspended_list, power.entry) {
		reinit_completion(&dev->power.completion);
		if (is_async(dev)) {
			get_device(dev);
			async_schedule(async_resume, dev);
		}
	}

	while (!list_empty(&dpm_suspended_list)) {
		dev = to_device(dpm_suspended_list.next);
		get_device(dev);
		if (!is_async(dev)) {
			int error;

			mutex_unlock(&dpm_list_mtx);

			error = device_resume(dev, state, false);
			if (error) {
				suspend_stats.failed_resume++;
				dpm_save_failed_step(SUSPEND_RESUME);
				dpm_save_failed_dev(dev_name(dev));
				pm_dev_err(dev, state, "", error);
			}

			mutex_lock(&dpm_list_mtx);
		}
		if (!list_empty(&dev->power.entry))
			list_move_tail(&dev->power.entry, &dpm_prepared_list);
		put_device(dev);
	}
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	dpm_show_time(starttime, state, 0, NULL);

	cpufreq_resume();
	trace_suspend_resume(TPS("dpm_resume"), state.event, false);
}

/**
 * device_complete - Complete a PM transition for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 */
static void device_complete(struct device *dev, pm_message_t state)
{
	void (*callback)(struct device *) = NULL;
	const char *info = NULL;

	if (dev->power.syscore)
		return;

	device_lock(dev);

	if (dev->pm_domain) {
		info = "completing power domain ";
		callback = dev->pm_domain->ops.complete;
	} else if (dev->type && dev->type->pm) {
		info = "completing type ";
		callback = dev->type->pm->complete;
	} else if (dev->class && dev->class->pm) {
		info = "completing class ";
		callback = dev->class->pm->complete;
	} else if (dev->bus && dev->bus->pm) {
		info = "completing bus ";
		callback = dev->bus->pm->complete;
	}

	if (!callback && dev->driver && dev->driver->pm) {
		info = "completing driver ";
		callback = dev->driver->pm->complete;
	}

	if (callback) {
		pm_dev_dbg(dev, state, info);
		callback(dev);
	}

	device_unlock(dev);

	pm_runtime_put(dev);
}

/**
 * dpm_complete - Complete a PM transition for all non-sysdev devices.
 * @state: PM transition of the system being carried out.
 *
 * Execute the ->complete() callbacks for all devices whose PM status is not
 * DPM_ON (this allows new devices to be registered).
 */
void dpm_complete(pm_message_t state)
{
	struct list_head list;

	trace_suspend_resume(TPS("dpm_complete"), state.event, true);
	might_sleep();

	INIT_LIST_HEAD(&list);
	mutex_lock(&dpm_list_mtx);
	while (!list_empty(&dpm_prepared_list)) {
		struct device *dev = to_device(dpm_prepared_list.prev);

		get_device(dev);
		dev->power.is_prepared = false;
		list_move(&dev->power.entry, &list);
		mutex_unlock(&dpm_list_mtx);

		trace_device_pm_callback_start(dev, "", state.event);
		device_complete(dev, state);
		trace_device_pm_callback_end(dev, 0);

		mutex_lock(&dpm_list_mtx);
		put_device(dev);
	}
	list_splice(&list, &dpm_list);
	mutex_unlock(&dpm_list_mtx);

	/* Allow device probing and trigger re-probing of deferred devices */
	device_unblock_probing();
	trace_suspend_resume(TPS("dpm_complete"), state.event, false);
}

/**
 * dpm_resume_end - Execute "resume" callbacks and complete system transition.
 * @state: PM transition of the system being carried out.
 *
 * Execute "resume" callbacks for all devices and complete the PM transition of
 * the system.
 */
void dpm_resume_end(pm_message_t state)
{
	dpm_resume(state);
	dpm_complete(state);
}
EXPORT_SYMBOL_GPL(dpm_resume_end);


/*------------------------- Suspend routines -------------------------*/

/**
 * resume_event - Return a "resume" message for given "suspend" sleep state.
 * @sleep_state: PM message representing a sleep state.
 *
 * Return a PM message representing the resume event corresponding to given
 * sleep state.
 */
static pm_message_t resume_event(pm_message_t sleep_state)
{
	switch (sleep_state.event) {
	case PM_EVENT_SUSPEND:
		return PMSG_RESUME;
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		return PMSG_RECOVER;
	case PM_EVENT_HIBERNATE:
		return PMSG_RESTORE;
	}
	return PMSG_ON;
}

/**
 * device_suspend_noirq - Execute a "late suspend" callback for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being suspended asynchronously.
 *
 * The driver of @dev will not receive interrupts while this function is being
 * executed.
 */
static int __device_suspend_noirq(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;

	TRACE_DEVICE(dev);
	TRACE_SUSPEND(0);

	dpm_wait_for_subordinate(dev, async);

	if (async_error)
		goto Complete;

	if (pm_wakeup_pending()) {
		async_error = -EBUSY;
		goto Complete;
	}

	if (dev->power.syscore || dev->power.direct_complete)
		goto Complete;

	if (dev->pm_domain) {
		info = "noirq power domain ";
		callback = pm_noirq_op(&dev->pm_domain->ops, state);
	} else if (dev->type && dev->type->pm) {
		info = "noirq type ";
		callback = pm_noirq_op(dev->type->pm, state);
	} else if (dev->class && dev->class->pm) {
		info = "noirq class ";
		callback = pm_noirq_op(dev->class->pm, state);
	} else if (dev->bus && dev->bus->pm) {
		info = "noirq bus ";
		callback = pm_noirq_op(dev->bus->pm, state);
	}

	if (!callback && dev->driver && dev->driver->pm) {
		info = "noirq driver ";
		callback = pm_noirq_op(dev->driver->pm, state);
	}

	error = dpm_run_callback(callback, dev, state, info);
	if (!error)
		dev->power.is_noirq_suspended = true;
	else
		async_error = error;

Complete:
	complete_all(&dev->power.completion);
	TRACE_SUSPEND(error);
	return error;
}

static void async_suspend_noirq(void *data, async_cookie_t cookie)
{
	struct device *dev = (struct device *)data;
	int error;

	error = __device_suspend_noirq(dev, pm_transition, true);
	if (error) {
		dpm_save_failed_dev(dev_name(dev));
		pm_dev_err(dev, pm_transition, " async", error);
	}

	put_device(dev);
}

static int device_suspend_noirq(struct device *dev)
{
	reinit_completion(&dev->power.completion);

	if (is_async(dev)) {
		get_device(dev);
		async_schedule(async_suspend_noirq, dev);
		return 0;
	}
	return __device_suspend_noirq(dev, pm_transition, false);
}

void dpm_noirq_begin(void)
{
	cpuidle_pause();
	device_wakeup_arm_wake_irqs();
	suspend_device_irqs();
}

int dpm_noirq_suspend_devices(pm_message_t state)
{
	ktime_t starttime = ktime_get();
	int error = 0;

	trace_suspend_resume(TPS("dpm_suspend_noirq"), state.event, true);
	mutex_lock(&dpm_list_mtx);
	pm_transition = state;
	async_error = 0;

	while (!list_empty(&dpm_late_early_list)) {
		struct device *dev = to_device(dpm_late_early_list.prev);

		get_device(dev);
		mutex_unlock(&dpm_list_mtx);

		error = device_suspend_noirq(dev);

		mutex_lock(&dpm_list_mtx);
		if (error) {
			pm_dev_err(dev, state, " noirq", error);
			dpm_save_failed_dev(dev_name(dev));
			put_device(dev);
			break;
		}
		if (!list_empty(&dev->power.entry))
			list_move(&dev->power.entry, &dpm_noirq_list);
		put_device(dev);

		if (async_error)
			break;
	}
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	if (!error)
		error = async_error;

	if (error) {
		suspend_stats.failed_suspend_noirq++;
		dpm_save_failed_step(SUSPEND_SUSPEND_NOIRQ);
	}
	dpm_show_time(starttime, state, error, "noirq");
	trace_suspend_resume(TPS("dpm_suspend_noirq"), state.event, false);
	return error;
}

/**
 * dpm_suspend_noirq - Execute "noirq suspend" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 *
 * Prevent device drivers' interrupt handlers from being called and invoke
 * "noirq" suspend callbacks for all non-sysdev devices.
 */
int dpm_suspend_noirq(pm_message_t state)
{
	int ret;

	dpm_noirq_begin();
	ret = dpm_noirq_suspend_devices(state);
	if (ret)
		dpm_resume_noirq(resume_event(state));

	return ret;
}

/**
 * device_suspend_late - Execute a "late suspend" callback for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being suspended asynchronously.
 *
 * Runtime PM is disabled for @dev while this function is being executed.
 */
static int __device_suspend_late(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;

	TRACE_DEVICE(dev);
	TRACE_SUSPEND(0);

	__pm_runtime_disable(dev, false);

	dpm_wait_for_subordinate(dev, async);

	if (async_error)
		goto Complete;

	if (pm_wakeup_pending()) {
		async_error = -EBUSY;
		goto Complete;
	}

	if (dev->power.syscore || dev->power.direct_complete)
		goto Complete;

	if (dev->pm_domain) {
		info = "late power domain ";
		callback = pm_late_early_op(&dev->pm_domain->ops, state);
	} else if (dev->type && dev->type->pm) {
		info = "late type ";
		callback = pm_late_early_op(dev->type->pm, state);
	} else if (dev->class && dev->class->pm) {
		info = "late class ";
		callback = pm_late_early_op(dev->class->pm, state);
	} else if (dev->bus && dev->bus->pm) {
		info = "late bus ";
		callback = pm_late_early_op(dev->bus->pm, state);
	}

	if (!callback && dev->driver && dev->driver->pm) {
		info = "late driver ";
		callback = pm_late_early_op(dev->driver->pm, state);
	}

	error = dpm_run_callback(callback, dev, state, info);
	if (!error)
		dev->power.is_late_suspended = true;
	else
		async_error = error;

Complete:
	TRACE_SUSPEND(error);
	complete_all(&dev->power.completion);
	return error;
}

static void async_suspend_late(void *data, async_cookie_t cookie)
{
	struct device *dev = (struct device *)data;
	int error;

	error = __device_suspend_late(dev, pm_transition, true);
	if (error) {
		dpm_save_failed_dev(dev_name(dev));
		pm_dev_err(dev, pm_transition, " async", error);
	}
	put_device(dev);
}

static int device_suspend_late(struct device *dev)
{
	reinit_completion(&dev->power.completion);

	if (is_async(dev)) {
		get_device(dev);
		async_schedule(async_suspend_late, dev);
		return 0;
	}

	return __device_suspend_late(dev, pm_transition, false);
}

/**
 * dpm_suspend_late - Execute "late suspend" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 */
int dpm_suspend_late(pm_message_t state)
{
	ktime_t starttime = ktime_get();
	int error = 0;

	trace_suspend_resume(TPS("dpm_suspend_late"), state.event, true);
	mutex_lock(&dpm_list_mtx);
	pm_transition = state;
	async_error = 0;

	while (!list_empty(&dpm_suspended_list)) {
		struct device *dev = to_device(dpm_suspended_list.prev);

		get_device(dev);
		mutex_unlock(&dpm_list_mtx);

		error = device_suspend_late(dev);

		mutex_lock(&dpm_list_mtx);
		if (!list_empty(&dev->power.entry))
			list_move(&dev->power.entry, &dpm_late_early_list);

		if (error) {
			pm_dev_err(dev, state, " late", error);
			dpm_save_failed_dev(dev_name(dev));
			put_device(dev);
			break;
		}
		put_device(dev);

		if (async_error)
			break;
	}
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	if (!error)
		error = async_error;
	if (error) {
		suspend_stats.failed_suspend_late++;
		dpm_save_failed_step(SUSPEND_SUSPEND_LATE);
		dpm_resume_early(resume_event(state));
	}
	dpm_show_time(starttime, state, error, "late");
	trace_suspend_resume(TPS("dpm_suspend_late"), state.event, false);
	return error;
}

/**
 * dpm_suspend_end - Execute "late" and "noirq" device suspend callbacks.
 * @state: PM transition of the system being carried out.
 */
int dpm_suspend_end(pm_message_t state)
{
	int error = dpm_suspend_late(state);
	if (error)
		return error;

	error = dpm_suspend_noirq(state);
	if (error) {
		dpm_resume_early(resume_event(state));
		return error;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(dpm_suspend_end);

/**
 * legacy_suspend - Execute a legacy (bus or class) suspend callback for device.
 * @dev: Device to suspend.
 * @state: PM transition of the system being carried out.
 * @cb: Suspend callback to execute.
 * @info: string description of caller.
 */
static int legacy_suspend(struct device *dev, pm_message_t state,
			  int (*cb)(struct device *dev, pm_message_t state),
			  const char *info)
{
	int error;
	ktime_t calltime;

	calltime = initcall_debug_start(dev);

	trace_device_pm_callback_start(dev, info, state.event);
	error = cb(dev, state);
	trace_device_pm_callback_end(dev, error);
	suspend_report_result(cb, error);

	initcall_debug_report(dev, calltime, error, state, info);

	return error;
}

static void dpm_clear_suppliers_direct_complete(struct device *dev)
{
	struct device_link *link;
	int idx;

	idx = device_links_read_lock();

	list_for_each_entry_rcu(link, &dev->links.suppliers, c_node) {
		spin_lock_irq(&link->supplier->power.lock);
		link->supplier->power.direct_complete = false;
		spin_unlock_irq(&link->supplier->power.lock);
	}

	device_links_read_unlock(idx);
}

/**
 * device_suspend - Execute "suspend" callbacks for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being suspended asynchronously.
 */
static int __device_suspend(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;
	char suspend_abort[MAX_SUSPEND_ABORT_LEN];
	DECLARE_DPM_WATCHDOG_ON_STACK(wd);

	TRACE_DEVICE(dev);
	TRACE_SUSPEND(0);

	dpm_wait_for_subordinate(dev, async);

	if (async_error) {
		dev->power.direct_complete = false;
		goto Complete;
	}

	/*
	 * Wait for possible runtime PM transitions of the device in progress
	 * to complete and if there's a runtime resume request pending for it,
	 * resume it before proceeding with invoking the system-wide suspend
	 * callbacks for it.
	 *
	 * If the system-wide suspend callbacks below change the configuration
	 * of the device, they must disable runtime PM for it or otherwise
	 * ensure that its runtime-resume callbacks will not be confused by that
	 * change in case they are invoked going forward.
	 */
	pm_runtime_barrier(dev);

	if (pm_wakeup_pending()) {
		pm_get_active_wakeup_sources(suspend_abort,
			MAX_SUSPEND_ABORT_LEN);
		log_suspend_abort_reason(suspend_abort);
		dev->power.direct_complete = false;
		async_error = -EBUSY;
		goto Complete;
	}

	if (dev->power.syscore)
		goto Complete;

	/* Avoid direct_complete to let wakeup_path propagate. */
	if (device_may_wakeup(dev) || dev->power.wakeup_path)
		dev->power.direct_complete = false;

	if (dev->power.direct_complete) {
		if (pm_runtime_status_suspended(dev)) {
			pm_runtime_disable(dev);
			if (pm_runtime_status_suspended(dev))
				goto Complete;

			pm_runtime_enable(dev);
		}
		dev->power.direct_complete = false;
	}

	dpm_watchdog_set(&wd, dev);
	device_lock(dev);

	if (dev->pm_domain) {
		info = "power domain ";
		callback = pm_op(&dev->pm_domain->ops, state);
		goto Run;
	}

	if (dev->type && dev->type->pm) {
		info = "type ";
		callback = pm_op(dev->type->pm, state);
		goto Run;
	}

	if (dev->class) {
		if (dev->class->pm) {
			info = "class ";
			callback = pm_op(dev->class->pm, state);
			goto Run;
		} else if (dev->class->suspend) {
			pm_dev_dbg(dev, state, "legacy class ");
			error = legacy_suspend(dev, state, dev->class->suspend,
						"legacy class ");
			goto End;
		}
	}

	if (dev->bus) {
		if (dev->bus->pm) {
			info = "bus ";
			callback = pm_op(dev->bus->pm, state);
		} else if (dev->bus->suspend) {
			pm_dev_dbg(dev, state, "legacy bus ");
			error = legacy_suspend(dev, state, dev->bus->suspend,
						"legacy bus ");
			goto End;
		}
	}

 Run:
	if (!callback && dev->driver && dev->driver->pm) {
		info = "driver ";
		callback = pm_op(dev->driver->pm, state);
	}

	error = dpm_run_callback(callback, dev, state, info);

 End:
	if (!error) {
		struct device *parent = dev->parent;

		dev->power.is_suspended = true;
		if (parent) {
			spin_lock_irq(&parent->power.lock);

			dev->parent->power.direct_complete = false;
			if (dev->power.wakeup_path
			    && !dev->parent->power.ignore_children)
				dev->parent->power.wakeup_path = true;

			spin_unlock_irq(&parent->power.lock);
		}
		dpm_clear_suppliers_direct_complete(dev);
	}

	device_unlock(dev);
	dpm_watchdog_clear(&wd);

 Complete:
	if (error)
		async_error = error;

	complete_all(&dev->power.completion);
	TRACE_SUSPEND(error);
	return error;
}

static void async_suspend(void *data, async_cookie_t cookie)
{
	struct device *dev = (struct device *)data;
	int error;

	error = __device_suspend(dev, pm_transition, true);
	if (error) {
		dpm_save_failed_dev(dev_name(dev));
		pm_dev_err(dev, pm_transition, " async", error);
	}

	put_device(dev);
}

static int device_suspend(struct device *dev)
{
	reinit_completion(&dev->power.completion);

	if (is_async(dev)) {
		get_device(dev);
		async_schedule(async_suspend, dev);
		return 0;
	}

	return __device_suspend(dev, pm_transition, false);
}

/**
 * dpm_suspend - Execute "suspend" callbacks for all non-sysdev devices.
 * @state: PM transition of the system being carried out.
 */
int dpm_suspend(pm_message_t state)
{
	ktime_t starttime = ktime_get();
	int error = 0;

	trace_suspend_resume(TPS("dpm_suspend"), state.event, true);
	might_sleep();

	cpufreq_suspend();

	mutex_lock(&dpm_list_mtx);
	pm_transition = state;
	async_error = 0;
	while (!list_empty(&dpm_prepared_list)) {
		struct device *dev = to_device(dpm_prepared_list.prev);

		get_device(dev);
		mutex_unlock(&dpm_list_mtx);

		error = device_suspend(dev);

		mutex_lock(&dpm_list_mtx);
		if (error) {
			pm_dev_err(dev, state, "", error);
			dpm_save_failed_dev(dev_name(dev));
			put_device(dev);
			break;
		}
		if (!list_empty(&dev->power.entry))
			list_move(&dev->power.entry, &dpm_suspended_list);
		put_device(dev);
		if (async_error)
			break;
	}
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	if (!error)
		error = async_error;
	if (error) {
		suspend_stats.failed_suspend++;
		dpm_save_failed_step(SUSPEND_SUSPEND);
	}
	dpm_show_time(starttime, state, error, NULL);
	trace_suspend_resume(TPS("dpm_suspend"), state.event, false);
	return error;
}

/**
 * device_prepare - Prepare a device for system power transition.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 *
 * Execute the ->prepare() callback(s) for given device.  No new children of the
 * device may be registered after this function has returned.
 */
static int device_prepare(struct device *dev, pm_message_t state)
{
	int (*callback)(struct device *) = NULL;
	int ret = 0;

	if (dev->power.syscore)
		return 0;

	/*
	 * If a device's parent goes into runtime suspend at the wrong time,
	 * it won't be possible to resume the device.  To prevent this we
	 * block runtime suspend here, during the prepare phase, and allow
	 * it again during the complete phase.
	 */
	pm_runtime_get_noresume(dev);

	device_lock(dev);

	dev->power.wakeup_path = device_may_wakeup(dev);

	if (dev->power.no_pm_callbacks) {
		ret = 1;	/* Let device go direct_complete */
		goto unlock;
	}

	if (dev->pm_domain)
		callback = dev->pm_domain->ops.prepare;
	else if (dev->type && dev->type->pm)
		callback = dev->type->pm->prepare;
	else if (dev->class && dev->class->pm)
		callback = dev->class->pm->prepare;
	else if (dev->bus && dev->bus->pm)
		callback = dev->bus->pm->prepare;

	if (!callback && dev->driver && dev->driver->pm)
		callback = dev->driver->pm->prepare;

	if (callback)
		ret = callback(dev);

unlock:
	device_unlock(dev);

	if (ret < 0) {
		suspend_report_result(callback, ret);
		pm_runtime_put(dev);
		return ret;
	}
	/*
	 * A positive return value from ->prepare() means "this device appears
	 * to be runtime-suspended and its state is fine, so if it really is
	 * runtime-suspended, you can leave it in that state provided that you
	 * will do the same thing with all of its descendants".  This only
	 * applies to suspend transitions, however.
	 */
	spin_lock_irq(&dev->power.lock);
	dev->power.direct_complete = ret > 0 && state.event == PM_EVENT_SUSPEND;
	spin_unlock_irq(&dev->power.lock);
	return 0;
}

/**
 * dpm_prepare - Prepare all non-sysdev devices for a system PM transition.
 * @state: PM transition of the system being carried out.
 *
 * Execute the ->prepare() callback(s) for all devices.
 */
int dpm_prepare(pm_message_t state)
{
	int error = 0;

	trace_suspend_resume(TPS("dpm_prepare"), state.event, true);
	might_sleep();

	/*
	 * Give a chance for the known devices to complete their probes, before
	 * disable probing of devices. This sync point is important at least
	 * at boot time + hibernation restore.
	 */
	wait_for_device_probe();
	/*
	 * It is unsafe if probing of devices will happen during suspend or
	 * hibernation and system behavior will be unpredictable in this case.
	 * So, let's prohibit device's probing here and defer their probes
	 * instead. The normal behavior will be restored in dpm_complete().
	 */
	device_block_probing();

	mutex_lock(&dpm_list_mtx);
	while (!list_empty(&dpm_list)) {
		struct device *dev = to_device(dpm_list.next);

		get_device(dev);
		mutex_unlock(&dpm_list_mtx);

		trace_device_pm_callback_start(dev, "", state.event);
		error = device_prepare(dev, state);
		trace_device_pm_callback_end(dev, error);

		mutex_lock(&dpm_list_mtx);
		if (error) {
			if (error == -EAGAIN) {
				put_device(dev);
				error = 0;
				continue;
			}
			printk(KERN_INFO "PM: Device %s not prepared "
				"for power transition: code %d\n",
				dev_name(dev), error);
			dpm_save_failed_dev(dev_name(dev));
			put_device(dev);
			break;
		}
		dev->power.is_prepared = true;
		if (!list_empty(&dev->power.entry))
			list_move_tail(&dev->power.entry, &dpm_prepared_list);
		put_device(dev);
	}
	mutex_unlock(&dpm_list_mtx);
	trace_suspend_resume(TPS("dpm_prepare"), state.event, false);
	return error;
}

/**
 * dpm_suspend_start - Prepare devices for PM transition and suspend them.
 * @state: PM transition of the system being carried out.
 *
 * Prepare all non-sysdev devices for system PM transition and execute "suspend"
 * callbacks for them.
 */
int dpm_suspend_start(pm_message_t state)
{
	int error;

	error = dpm_prepare(state);
	if (error) {
		suspend_stats.failed_prepare++;
		dpm_save_failed_step(SUSPEND_PREPARE);
	} else
		error = dpm_suspend(state);
	return error;
}
EXPORT_SYMBOL_GPL(dpm_suspend_start);

void __suspend_report_result(const char *function, void *fn, int ret)
{
	if (ret)
		printk(KERN_ERR "%s(): %pF returns %d\n", function, fn, ret);
}
EXPORT_SYMBOL_GPL(__suspend_report_result);

/**
 * device_pm_wait_for_dev - Wait for suspend/resume of a device to complete.
 * @dev: Device to wait for.
 * @subordinate: Device that needs to wait for @dev.
 */
int device_pm_wait_for_dev(struct device *subordinate, struct device *dev)
{
	dpm_wait(dev, subordinate->power.async_suspend);
	return async_error;
}
EXPORT_SYMBOL_GPL(device_pm_wait_for_dev);

/**
 * dpm_for_each_dev - device iterator.
 * @data: data for the callback.
 * @fn: function to be called for each device.
 *
 * Iterate over devices in dpm_list, and call @fn for each device,
 * passing it @data.
 */
void dpm_for_each_dev(void *data, void (*fn)(struct device *, void *))
{
	struct device *dev;

	if (!fn)
		return;

	device_pm_lock();
	list_for_each_entry(dev, &dpm_list, power.entry)
		fn(dev, data);
	device_pm_unlock();
}
EXPORT_SYMBOL_GPL(dpm_for_each_dev);

static bool pm_ops_is_empty(const struct dev_pm_ops *ops)
{
	if (!ops)
		return true;

	return !ops->prepare &&
	       !ops->suspend &&
	       !ops->suspend_late &&
	       !ops->suspend_noirq &&
	       !ops->resume_noirq &&
	       !ops->resume_early &&
	       !ops->resume &&
	       !ops->complete;
}

void device_pm_check_callbacks(struct device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->power.lock, flags);
	dev->power.no_pm_callbacks =
		(!dev->bus || (pm_ops_is_empty(dev->bus->pm) &&
		 !dev->bus->suspend && !dev->bus->resume)) &&
		(!dev->class || (pm_ops_is_empty(dev->class->pm) &&
		 !dev->class->suspend && !dev->class->resume)) &&
		(!dev->type || pm_ops_is_empty(dev->type->pm)) &&
		(!dev->pm_domain || pm_ops_is_empty(&dev->pm_domain->ops)) &&
		(!dev->driver || (pm_ops_is_empty(dev->driver->pm) &&
		 !dev->driver->suspend && !dev->driver->resume));
	spin_unlock_irqrestore(&dev->power.lock, flags);
}
