Für Ubuntu 20.04.6 (LTS):
- Python 3.8.10 und pip 20.0.2 installieren: sudo apt install python3-pip
- (vielleicht nicht notwendig) export PATH="/home/USERNAME/.local/bin/:$PATH" (tatsächlicher Benutzername anstatt USERNAME)
- Das Repo herunterladen: https://github.com/mahaarbo/urdf2casadi [master], commit fc4232d (on Aug 3, 2022)
- In das Verzeichnis urdf2casadi wechseln
- pip install --user .

Funktioniert auch unter Windows in wsl2 mit Ubuntu 20.04.6 (LTS) aus dem Microsoft Store.