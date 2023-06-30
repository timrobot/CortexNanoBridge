To use this package, use pip to install via wheel:

```bash
python3 -m pip install .
```

Then run the supporting scripts that will enable USB serial for $USER and autostart the worker.py script on launch

```bash
./enable-tty.sh
./enable-autostart.sh
```