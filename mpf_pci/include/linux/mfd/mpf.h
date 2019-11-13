/* SPDX-License-Identifier: LGPL-2.0 */

#ifndef __MPF_H
#define __MPF_H

struct mpf_device {
	struct pci_dev *pdev;
	struct device *dev;
	struct cdev cdev;
	void __iomem *csr;
	struct regmap *regmap;
	int num_msi_irqs;
	int irq_base;
	struct irq_domain *irq_domain;
	bool is_swifd;
#ifdef CONFIG_PM_SLEEP
	struct notifier_block pm_notify;
#endif /* CONFIG_PM_SLEEP */
};

#endif
