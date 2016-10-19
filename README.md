# tesi-Letizia
Repository con i files della tesi di Letizia Curti

branch: nuovo sistema

NEW: la mappa dell'ambiente è in LEMON


*nuovi nodi:
-m nodi robot

-master: riceve il vettore dei task da assegnare dal task_manager, pubblica gli assignment al motion_planner, considerando anche le info aggiornate che riceve dallo stesso motion_planner

-motion_planner: riceve gli assignments dal central_node, riceve imprevisti dal nodo_imprevisti che incideranno sullo stato dei robot, monitora lo stato dei robot e manda le info aggiornate al central_node

-obstacle_node: manda imprevisti al motion planner (es. presenza di ostacoli o di altri robot), serve per simulare situazioni reali particolari (da aggiungere: legge da file yaml gli imprevisti)

-task_manager: legge i nuovi task dal file yaml task_config.yaml e tiene traccia di quelli che vengono completati (pubblicati dal motion_planner), pubblica al central_node il vettore degli n(t) task da assegnare


*pubblicazione dei ref frame in tf

*visualizzazione su rviz svincolata da turtlesim

