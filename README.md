# Projet PER 2025-2026 Contrôle de vol d'un nano drone et détection de dérive comportementale

## Description
Le projet vise à exploiter le Crazyflie 2.1+ de Bitcraze, qui est un nano-drone modulaire et open-source. Équipé du Flow v2 Deck pour l’estimation de mouvement et du Multi-ranger Deck pour la détection d’obstacles, le drone permettra de concevoir et tester des stratégies de pilotage autonome en environnement 3D perturbé (obstacles, ...) mais aussi d’analyser les écarts entre comportement attendu et observé.
L’objectif principal est de développer, sous ROS 2 Humble et Gazebo, deux nœuds principaux : le premier va gérer la génération et le suivi de trajectoires, tandis que le second analysera les écarts entre ce qui est demandé au drone et ce qui se passe réellement afin de détecter toute dérive comportementale du système. 
