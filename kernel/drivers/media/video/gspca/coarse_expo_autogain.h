

static int gspca_coarse_grained_expo_autogain(struct gspca_dev *gspca_dev,
	int avg_lum, int desired_avg_lum, int deadzone)
{
	int i, steps, gain, orig_gain, exposure, orig_exposure;
	int gain_low, gain_high;
	const struct ctrl *gain_ctrl = NULL;
	const struct ctrl *exposure_ctrl = NULL;
	struct sd *sd = (struct sd *) gspca_dev;
	int retval = 0;

	for (i = 0; i < gspca_dev->sd_desc->nctrls; i++) {
		if (gspca_dev->ctrl_dis & (1 << i))
			continue;
		if (gspca_dev->sd_desc->ctrls[i].qctrl.id == V4L2_CID_GAIN)
			gain_ctrl = &gspca_dev->sd_desc->ctrls[i];
		if (gspca_dev->sd_desc->ctrls[i].qctrl.id == V4L2_CID_EXPOSURE)
			exposure_ctrl = &gspca_dev->sd_desc->ctrls[i];
	}
	if (!gain_ctrl || !exposure_ctrl) {
		PDEBUG(D_ERR, "Error: gspca_coarse_grained_expo_autogain "
			"called on cam without gain or exposure");
		return 0;
	}

	if (gain_ctrl->get(gspca_dev, &gain) ||
	    exposure_ctrl->get(gspca_dev, &exposure))
		return 0;

	orig_gain = gain;
	orig_exposure = exposure;
	gain_low =
		(gain_ctrl->qctrl.maximum - gain_ctrl->qctrl.minimum) / 5 * 2;
	gain_low += gain_ctrl->qctrl.minimum;
	gain_high =
		(gain_ctrl->qctrl.maximum - gain_ctrl->qctrl.minimum) / 5 * 4;
	gain_high += gain_ctrl->qctrl.minimum;

	/* If we are of a multiple of deadzone, do multiple steps to reach the
	   desired lumination fast (with the risc of a slight overshoot) */
	steps = (desired_avg_lum - avg_lum) / deadzone;

	PDEBUG(D_FRAM, "autogain: lum: %d, desired: %d, steps: %d",
		avg_lum, desired_avg_lum, steps);

	if ((gain + steps) > gain_high &&
	    sd->exposure < exposure_ctrl->qctrl.maximum) {
		gain = gain_high;
		sd->exp_too_low_cnt++;
	} else if ((gain + steps) < gain_low &&
		   sd->exposure > exposure_ctrl->qctrl.minimum) {
		gain = gain_low;
		sd->exp_too_high_cnt++;
	} else {
		gain += steps;
		if (gain > gain_ctrl->qctrl.maximum)
			gain = gain_ctrl->qctrl.maximum;
		else if (gain < gain_ctrl->qctrl.minimum)
			gain = gain_ctrl->qctrl.minimum;
		sd->exp_too_high_cnt = 0;
		sd->exp_too_low_cnt = 0;
	}

	if (sd->exp_too_high_cnt > 3) {
		exposure--;
		sd->exp_too_high_cnt = 0;
	} else if (sd->exp_too_low_cnt > 3) {
		exposure++;
		sd->exp_too_low_cnt = 0;
	}

	if (gain != orig_gain) {
		gain_ctrl->set(gspca_dev, gain);
		retval = 1;
	}
	if (exposure != orig_exposure) {
		exposure_ctrl->set(gspca_dev, exposure);
		retval = 1;
	}

	return retval;
}
