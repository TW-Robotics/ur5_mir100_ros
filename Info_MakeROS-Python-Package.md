### INFO ###
Neue Python-Packages in ROS erstellen und ausführen:
- Ordner unter src mit Packagename anlegen (project/src/myPackage)
- In diesem Ordner ein leeres File __init__.py anlegen
- In diesem Ordner das eigentlich .py-File anlegen und programmieren
- Dem eigentlichen .py-File Rechte Zuweisen (Datei als Programm ausführen)
- Im setup.py-File (project/setup.py) den Packagenamen angeben
- Im bin-Ordner (project/bin) ein File mit dem Packagenamen ("myPackage") anlegen und in der main dieses Files das eigentliche .py-File angeben (Dieses wird ausgeführt, wenn man das Package ruft)
- catkin_make ausführen
- Ausführen mit rosrun project myPackage --> Es wird in setup.py-File nachgeschaut, ob es das Package gibt, das File im bin-Ordner gerufen und dieses startet dann das eigentliche .py-File im src-Ordner
