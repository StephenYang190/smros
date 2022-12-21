# install oh my zsh
sudo apt update
sudo apt install -y zsh curl openjdk-8-jdk
echo y | sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
echo 'zsh' >> "/home/${DOCKER_USER}/.bashrc"
zsh

