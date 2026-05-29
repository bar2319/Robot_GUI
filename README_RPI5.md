# Raspberry Pi 5 Transfer Package (Ubuntu)

This folder contains the runtime files needed to run the motor GUI on Raspberry Pi 5.

## Included
- motor_gui.py
- helpers.py
- steadywin_can.py
- tab_single_motor.py
- tab_pid.py
- tab_all_motors.py
- tab_locomotion.py
- tab_one_leg.py
- odrive-config-B340026C334D.json
- requirements.txt
- run_motor_gui.sh

## 1) Copy to Raspberry Pi
Example with scp from your current machine:

```bash
scp -r raspi5_transfer_package <pi_user>@<pi_ip>:~/
```

## 2) Install dependencies on Raspberry Pi (Ubuntu)

```bash
sudo apt update
sudo apt install -y python3-pip python3-pyqt5
cd ~/raspi5_transfer_package
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

## 3) CAN device permissions (if needed)
If using slcan on /dev/ttyACM0, ensure your user can access serial devices:

```bash
sudo usermod -aG dialout $USER
```

Log out and log in again after changing groups.

## 4) Run

```bash
cd ~/raspi5_transfer_package
bash run_motor_gui.sh
```

## Notes
- No control logic was changed in this package; these are copies.
- In the GUI connection bar, set Interface/Channel to match your Pi setup.
