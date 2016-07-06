# tesi-Letizia
Repository con i files della tesi di Letizia Curti

branch: nuovo sistema


*nuovi nodi:
-m nodi robot

-master: riceve il vettore dei task da assegnare dal task_manager, pubblica gli assignment al motion_planner, considerando anche le info aggiornate che riceve dallo stesso motion_planner

-motion_planner: riceve gli assignments dal central_node, riceve imprevisti dal nodo_imprevisti che incideranno sullo stato dei robot, monitora lo stato dei robot e manda le info aggiornate al central_node

-obstacle_node: manda imprevisti al motion planner (es. presenza di ostacoli o di altri robot), serve per simulare situazioni reali particolari (da aggiungere: legge da file yaml gli imprevisti)

-task_manager: tiene traccia dei nuovi task (pubblicati dal nodo_utenti) e di quelli che vengono completati (pubblicati dal motion_planner), pubblica al central_node il vettore degli n(t) task da assegnare

-users_node: manda i nuovi task richiesti (coppie di task) al task_manager (da aggiungere: legge da file yaml i nuovi task)


*nuovi msgs:

-task.msg: contiene le info relative ad una coppia di task

-vect_task.msg: è un vettore di coppie di task



*pubblicazione dei ref frame in tf

*visualizzazione su rviz svincolata da turtlesim

