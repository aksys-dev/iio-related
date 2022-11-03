#include <linux/ctype.h>
#include <linux/input.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <asm/unaligned.h>

struct adc_joystick_axis {
	u32 code;
	s32 range[2];
	s32 fuzz;
	s32 flat;
};

struct adc_joystick {
	struct input_dev *input;
	struct iio_cb_buffer *buffer;
	struct adc_joystick_axis *axes;
	struct iio_channel *chans;
	int num_chans;
};

static int adc_joystick_handle(const void *data, void *private)
{
	struct adc_joystick *joy = private;
	enum iio_endian endianness;
	int bytes, msb, val, idx, i;
	const u16 *data_u16;
	bool sign;

	bytes = joy->chans[0].channel->scan_type.storagebits >> 3;

	for (i = 0; i < joy->num_chans; ++i) {
		idx = joy->chans[i].channel->scan_index;
		endianness = joy->chans[i].channel->scan_type.endianness;
		msb = joy->chans[i].channel->scan_type.realbits - 1;
		sign = tolower(joy->chans[i].channel->scan_type.sign) == 's';

		switch (bytes) {
		case 1:
			val = ((const u8 *)data)[idx];
			break;
		case 2:
			data_u16 = (const u16 *)data + idx;

			/*
			 * Data is aligned to the sample size by IIO core.
			 * Call `get_unaligned_xe16` to hide type casting.
			 */
			if (endianness == IIO_BE)
				val = get_unaligned_be16(data_u16);
			else if (endianness == IIO_LE)
				val = get_unaligned_le16(data_u16);
			else /* IIO_CPU */
				val = *data_u16;
			break;
		default:
			return -EINVAL;
		}

		val >>= joy->chans[i].channel->scan_type.shift;
		if (sign)
			val = sign_extend32(val, msb);
		else
			val &= GENMASK(msb, 0);
		input_report_abs(joy->input, joy->axes[i].code, val);
	}

	input_sync(joy->input);

	return 0;
}

static int adc_joystick_open(struct input_dev *dev)
{
	struct adc_joystick *joy = input_get_drvdata(dev);
	struct device *devp = &dev->dev;
	int ret = 0;
	struct iio_dev *indio_dev;
	dev_err(devp, "%s(%d) start \n", __FUNCTION__, __LINE__);

	if (joy->buffer == NULL) {
		dev_err(devp, "%s(%d) cb_buffer == NULL\n", __FUNCTION__, __LINE__);
		return -22;
	}
	indio_dev = iio_channel_cb_get_iio_dev(joy->buffer);

	dev_err(devp, "%s(%d) joy->buferr=%p, iio_dev=%p\n", __FUNCTION__, __LINE__, joy->buffer, indio_dev);

#if 1
	ret = iio_channel_start_all_cb(joy->buffer);
	if (ret) {
		dev_err(devp, "%s(%d) Unable to start callback buffer-> %d\n", __FUNCTION__, __LINE__, ret);
	} else {
		dev_err(devp, "%s(%d) iio_channel_start_all_cb OK!\n", __FUNCTION__, __LINE__);
	}
#endif
	return ret;
}

static void adc_joystick_close(struct input_dev *dev)
{
	struct adc_joystick *joy = input_get_drvdata(dev);
	struct device *devp = &dev->dev;
	dev_err(devp, "%s(%d) start \n", __FUNCTION__, __LINE__);
	iio_channel_stop_all_cb(joy->buffer);
	dev_err(devp, "%s(%d) stop \n", __FUNCTION__, __LINE__);
}

static void adc_joystick_cleanup(void *data)
{
	iio_channel_release_all_cb(data);
}

static int adc_joystick_set_axes(struct device *dev, struct adc_joystick *joy)
{
	struct adc_joystick_axis *axes;
	struct fwnode_handle *child;
	int num_axes, error, i;

	num_axes = device_get_child_node_count(dev);
	if (!num_axes) {
		dev_err(dev, "Unable to find child nodes\n");
		return -EINVAL;
	} else {
		dev_err(dev, "%s(%d) find child nodes %d!\n",__FUNCTION__, __LINE__, num_axes);
	}

	if (num_axes != joy->num_chans) {
		dev_err(dev, "Got %d child nodes for %d channels\n",
			num_axes, joy->num_chans);
		return -EINVAL;
	} else {
		dev_err(dev, "%s(%d) Got %d child nodes for %d channels\n",__FUNCTION__, __LINE__, num_axes, joy->num_chans);
	}

	axes = devm_kmalloc_array(dev, num_axes, sizeof(*axes), GFP_KERNEL);
	if (!axes) {
		return -ENOMEM;
	} else {
		dev_err(dev, "%s(%d) devm_kmalloc_array OK!\n",__FUNCTION__, __LINE__);
	}

	device_for_each_child_node(dev, child) {
		error = fwnode_property_read_u32(child, "reg", &i);
		if (error) {
			dev_err(dev, "reg invalid or missing\n");
			goto err_fwnode_put;
		} else {
			dev_err(dev, "%s(%d) reg node OK!\n",__FUNCTION__, __LINE__);
		}

		if (i >= num_axes) {
			error = -EINVAL;
			dev_err(dev, "No matching axis for reg %d\n", i);
			goto err_fwnode_put;
		} else {
			dev_err(dev, "%s(%d) %d matching axis for reg!\n",__FUNCTION__, __LINE__, i);
		}

		error = fwnode_property_read_u32(child, "linux,code",
						 &axes[i].code);
		if (error) {
			dev_err(dev, "linux,code invalid or missing\n");
			goto err_fwnode_put;
		} else {
			dev_err(dev, "%s(%d) linux,code = %d!\n",__FUNCTION__, __LINE__, axes[i].code);
		}

		error = fwnode_property_read_u32_array(child, "abs-range",
						       axes[i].range, 2);
		if (error) {
			dev_err(dev, "abs-range invalid or missing\n");
			goto err_fwnode_put;
		} else {
			//dev_err(dev, "%s(%d) abs-range= %d ~ %d !\n",__FUNCTION__, __LINE__, axes[i].range[1], axes[i].range[2]);
			dev_err(dev, "%s(%d) abs-range!\n",__FUNCTION__, __LINE__);
		}

		fwnode_property_read_u32(child, "abs-fuzz", &axes[i].fuzz);
		fwnode_property_read_u32(child, "abs-flat", &axes[i].flat);
		dev_err(dev, "%s(%d) abs-fuzz= %d abs-flat=%d !\n",__FUNCTION__, __LINE__, axes[i].fuzz, axes[i].flat);
		input_set_abs_params(joy->input, axes[i].code,
				     axes[i].range[0], axes[i].range[1],
				     axes[i].fuzz, axes[i].flat);
		input_set_capability(joy->input, EV_ABS, axes[i].code);
	}

	joy->axes = axes;

	return 0;

err_fwnode_put:
	fwnode_handle_put(child);
	return error;
}

static int adc_joystick_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adc_joystick *joy;
	struct input_dev *input;
	struct iio_cb_buffer *cb_buffer;
	int error;
	int bits;
	int i;
	//int ret;

	joy = devm_kzalloc(dev, sizeof(*joy), GFP_KERNEL);
	if (!joy) {
		dev_err(dev, "%s(%d) devm_kzalloc failed!\n",__FUNCTION__, __LINE__);
		return -ENOMEM;
	} else {
		dev_err(dev, "%s(%d) devm_kzalloc OK!\n",__FUNCTION__, __LINE__);
	}

	joy->chans = devm_iio_channel_get_all(dev);
	if (IS_ERR(joy->chans)) {
		error = PTR_ERR(joy->chans);
		if (error != -EPROBE_DEFER)
			dev_err(dev, "Unable to get IIO channels");
		return error;
	} else {
		dev_err(dev, "%s(%d) devm_iio_channel_get_all OK!\n",__FUNCTION__, __LINE__);
	}

	/* Count how many channels we got. NULL terminated. */
	for (i = 0; joy->chans[i].indio_dev; i++) {
		dev_err(dev, "%s(%d) dev name: [%p]%s bits\n",__FUNCTION__, __LINE__, joy->chans[i].indio_dev->channels, joy->chans[i].indio_dev->name);
		bits = joy->chans[i].channel->scan_type.storagebits;
		if (!bits || bits > 16) {
			dev_err(dev, "Unsupported channel storage size\n");
			return -EINVAL;
		} else {
			dev_err(dev, "%s(%d) channel storage size: %d bits\n",__FUNCTION__, __LINE__, bits);
		}
		if (bits != joy->chans[0].channel->scan_type.storagebits) {
			dev_err(dev, "Channels must have equal storage size\n");
			return -EINVAL;
		}
	}
	joy->num_chans = i;

	dev_err(dev, "%s(%d) joy->num_chans = %d\n",__FUNCTION__, __LINE__, joy->num_chans);

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "Unable to allocate input device\n");
		return -ENOMEM;
	} else {
		dev_err(dev, "%s(%d) devm_input_allocate_device OK. input name is: %s\n",__FUNCTION__, __LINE__, pdev->name);
	}

	joy->input = input;
	input->name = pdev->name;
	input->id.bustype = BUS_HOST;
	input->open = adc_joystick_open;
	input->close = adc_joystick_close;


	error = adc_joystick_set_axes(dev, joy);
	if (error) {
		return error;
	} else {
		dev_err(dev, "%s(%d) adc_joystick_set_axes OK\n",__FUNCTION__, __LINE__);
	}

	input_set_drvdata(input, joy);
	
	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device\n");
		return error;
	} else {
		dev_err(dev, "%s(%d) input_register_device input->name=%s, input->phys=%s, input->uniq=%s\n",__FUNCTION__, __LINE__, input->name, input->phys, input->uniq);
	}

	joy->buffer = iio_channel_get_all_cb(dev, adc_joystick_handle, joy);
	if (IS_ERR(joy->buffer)) {
		error = PTR_ERR(joy->buffer);
		dev_err(dev, "Unable to allocate callback buffer, code = %d\n", error);
		return error;
	} else {
		dev_err(dev, "%s(%d) iio_channel_get_all_cb OK\n",__FUNCTION__, __LINE__);
	}
	cb_buffer = joy->buffer;

	dev_err(dev, "%s(%d) joy->buferr=%p \n",__FUNCTION__, __LINE__, joy->buffer);

	error = devm_add_action_or_reset(dev, adc_joystick_cleanup, joy->buffer);
	if (error)  {
		dev_err(dev, "Unable to add action\n");
		return error;
	} else {
		dev_err(dev, "%s(%d) devm_add_action_or_reset OK\n",__FUNCTION__, __LINE__);
	}

	dev_err(dev, "%s(%d) joy->buferr=%p\n", __FUNCTION__, __LINE__, joy->buffer);
#if 0
	ret = iio_channel_start_all_cb(joy->buffer);
	if (ret) {
		dev_err(dev, "%s(%d) Unable to start callback buffer-> %d\n", __FUNCTION__, __LINE__, ret);
	} else {
		dev_err(dev, "%s(%d) iio_channel_start_all_cb OK!\n", __FUNCTION__, __LINE__);
	}
#endif
	return 0;
}

static const struct of_device_id adc_joystick_of_match[] = {
	{ .compatible = "adc-joystick", },
	{ }
};
MODULE_DEVICE_TABLE(of, adc_joystick_of_match);

static struct platform_driver adc_joystick_driver = {
	.driver = {
		.name = "adc-joystick",
		.of_match_table = adc_joystick_of_match,
	},
	.probe = adc_joystick_probe,
};
module_platform_driver(adc_joystick_driver);

MODULE_DESCRIPTION("Input driver for joysticks connected over ADC");
MODULE_AUTHOR("Artur Rojek <contact@artur-rojek.eu>");
MODULE_LICENSE("GPL");
