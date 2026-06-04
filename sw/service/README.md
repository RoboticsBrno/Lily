# Bear Rescue GPIO Service

Service for controlling the bear rescue program via buttons.

## Installation

Run these commands on the Pi (adapt paths if your checkout is somewhere other than `~/Lily`):

```bash
mkdir -p ~/.config/systemd/user
ln -s ~/Lily/sw/systemd/bear-rescue.service ~/.config/systemd/user/bear-rescue.service
loginctl enable-linger $USER
systemctl --user daemon-reload
systemctl --user enable bear-rescue.service
systemctl --user start bear-rescue.service
```
