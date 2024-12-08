# Installing Tools
## Installing VSCode
first install vscode using snap
```bash
sudo snap install code --classic
```
then add the `ROS` extension from Microsoft.

## Installing Colcon
To install colcon first run the sudo apt install command.
``` bash
sudo apt install python3-colcon-common-extensions
```

then add the colcon autocomplete to the `.bashrc` file.
``` bash
echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc
```

