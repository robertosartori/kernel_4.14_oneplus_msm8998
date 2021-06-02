
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include "fingerprint_detect.h"

static ssize_t sensor_version_get(struct device *device,
			     struct device_attribute *attribute,
			     char *buffer)
{
	return scnprintf(buffer, PAGE_SIZE, "%i\n", 0x01);
}

static DEVICE_ATTR(sensor_version, S_IRUSR, sensor_version_get, NULL);

static struct attribute *attributes[] = {
	&dev_attr_sensor_version.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static int fingerprint_detect_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device *dev = &pdev->dev;

	struct fingerprint_detect_data *fp_detect =
		devm_kzalloc(dev, sizeof(*fp_detect),
			GFP_KERNEL);
	if (!fp_detect) {
		dev_err(dev,
			"failed to allocate memory for struct fingerprint_detect_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

exit:
	return rc;
}


static const struct of_device_id fingerprint_detect_of_match[] = {
	{ .compatible = "oneplus,fpdetect", },
	{}
};
MODULE_DEVICE_TABLE(op, fingerprint_detect_of_match);

static struct platform_driver fingerprint_detect_driver = {
	.driver = {
		.name		= "fingerprint_detect",
		.owner		= THIS_MODULE,
		.of_match_table = fingerprint_detect_of_match,
	},
	.probe = fingerprint_detect_probe,
};
module_platform_driver(fingerprint_detect_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("yale liu");
MODULE_DESCRIPTION("Fingerprint detect device driver.");
