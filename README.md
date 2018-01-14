# IMUforBinoculars

Le système se compose d’une centrale inertielle 9 axes, d’un processeur de calcul, et d’un PC ou sont installés le logiciel Stellarium.

En utilisant le GPS, nous obtenons la data, heure et position sur terre de l'observateur. En utilisant la centrale intertielle 9 axes nous obtenons la direction visée.

Après un calcul de changement de repère, l'ascention droite et la déclinaison sont envoyées au logiciel Stellarium qui montre l'endroit visé dans le ciel.

![Principe général](https://github.com/ddieffen/IMUforBinoculars/raw/master/Wiki/GeneralPrinciple.png)

Grâce à l'apparition de modules simples à utiliser, le câblage est relativement simple. Il suffit de connecter la centrale inertielle et le microcontrolleur au moyen d'un bus I2C et de connecter le GPS au moyen d'un port série.

![Principe général](https://github.com/ddieffen/IMUforBinoculars/raw/master/Schematics/StarPointer_bb.png)

Le plugin "Telescope Control" ou "Contôle de telescope" se charge d'envoyer des requêtes au microcontrolleur qui lui, répond en envoyant les coordonées visées actuellement.

[![Projection dans Stellarium](https://img.youtube.com/vi/Eth5bdYiN0c/0.jpg)](https://www.youtube.com/watch?v=Eth5bdYiN0c)

