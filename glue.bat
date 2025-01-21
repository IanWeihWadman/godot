cd /D "%~dp0"
scons p=windows tools=yes module_mono_enabled=yes mono_glue=no
bin\godot.windows.editor.x86_64.mono.exe --generate-mono-glue modules/mono/glue
modules\mono\build_scripts\build_assemblies.py --godot-output-dir bin --godot-platform=windows --push-nupkgs-local D:\Godot++\GodotLocalNuget
scons p=windows tools=yes module_mono_enabled=yes mono_glue=yes