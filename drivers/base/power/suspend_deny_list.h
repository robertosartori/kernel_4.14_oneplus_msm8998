#ifndef SUSPEND_DENY_LIST_H
#define SUSPEND_DENY_LIST_H

static const char *suspend_deny_list[] = {
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
	"5065000.qcom,gpucc",
	"179c0000.qcom,cpu-clock-8998",
	"179c0000.qcom,cpu-clock-8998:qcom,limits-dcvs@179ce800",
	"179c0000.qcom,cpu-clock-8998:qcom,limits-dcvs@179cc800",
	"5066094.qcom,gdsc",
	"regulator.74",
	"soc:qcom,msm-cpufreq",
};

#endif // SUSPEND_DENY_LIST_H
