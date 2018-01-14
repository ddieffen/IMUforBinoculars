# IMUforBinoculars

Le système se compose d’une centrale inertielle 9 axes, d’un processeur de calcul, et d’un PC ou sont installés le logiciel Stellarium.

En utilisant le GPS, nous obtenons la data, heure et position sur terre de l'observateur. En utilisant la centrale intertielle 9 axes nous obtenons la direction visée.

Après un calcul de changement de repère, l'ascention droite et la déclinaison sont envoyées au logiciel Stellarium qui montre l'endroit visé dans le ciel.

![Principe général](https://github.com/ddieffen/IMUforBinoculars/raw/master/Wiki/GeneralPrinciple.png)