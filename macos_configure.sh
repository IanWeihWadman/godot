/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

brew install scons
brew install xcodes
brew install dotnet

/bin/bash misc/scripts/install_vulkan_sdk_macos.sh

xcodes install 16.3
sudo xcode-select -s /Applications/Xcode-16.3.0.app/Contents/Developer

mkdir ~/LocalNuget
dotnet nuget add source ~/LocalNuget --name LocalNuget
./modules/mono/build_scripts/build_assemblies.py --godot-output-dir ./bin --push-nupkgs-local ~/LocalNuget