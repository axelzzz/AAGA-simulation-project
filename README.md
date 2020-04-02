Notre stratégie consiste à faire déplacer nos Main tout droit simultanément à partir du centre de l'arène tout en 
tirant en rafale. Ils tirent par défaut tout droit lorsque les Secondary ne communiquent pas de position d'ennemis.

Les 3 Main sont séparés d'une petite distance de telle sorte que lorsqu'ils attaquent, ils attaquent la même cible, 
pour la neutraliser le plus rapidement possible. Lorsqu'un ennemi est détecté par les Secondary, les Mains s'arrêtent
et attaquent la cible. Les Secondary se déplacent dans ce cas-là pour continuer la détection.

Nous avons placé un Secondary par couloir entourant les Main. Ils sont placés à équidistance des Mains de telle sorte 
que leur rayon de détection puisse scanner un rayon environ égal à la moitié de la hauteur du terrain.
Ils se déplacent plus rapidement que les Main puis exécutent des aller retour pour ne pas rester statiques 
et tenter d'esquiver des attaques ennemies. Dans le pire des cas, ils peuvent servir de bouclier pour les Main. 
Ils peuvent également détecter les balles alliées pour tenter de les éviter. 
Arrivé au bout du terrain, les Secondary et les mains refont la même stratégie en reculant.

Nos 5 robots communiquent entre eux pour connaitre leurs positions respectives. Ce calcul de position est effectué
avant de tirer pour éviter les tirs alliés.

Si des épaves empêchent le déplacement de nos bots, ils les contournent.

Lorsqu'un Secondary trouve un ennemi, il envoie la position (de l'ennemi) aux Main puis recule pour ne pas être détécté par 
l'ennemi.
