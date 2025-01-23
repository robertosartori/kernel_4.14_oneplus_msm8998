#ifndef SUSPEND_DENY_LIST_H
#define SUSPEND_DENY_LIST_H

static const char *suspend_deny_list[] = {
	"100000.qcom,gcc",
	"c8c0000.qcom,mmsscc",
	"5065000.qcom,early_gpucc",
	"179c8000.cprh-ctrl",
	"179c4000.cprh-ctrl",
	"5061000.cpr4-ctrl",
	"c8ce020.qcom,gdsc",
	"5065000.qcom,gpucc",
	"5066094.qcom,gdsc",
};

#endif // SUSPEND_DENY_LIST_H
