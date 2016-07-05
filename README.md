# tesi-Letizia
Repository con i files della tesi di Letizia Curti

branch: modifiche


*modificati i msgs:

-i task e i robot pubblicano un nuovo msg (IniStatus) sui rispettivi topic

-il central node pubblica un nuovo msg (AssignMsg) che contiene il vettore degli assignment (ciò che prima stava in map assignment)

-il central node e i robot nodes si scambiano i "free messages" su un nuovo topic (free_assign_topic) e non più su assignment topic


*pubblicazione dei ref frame in tf

*visualizzazione su rviz svincolata da turtlesim

*i task arrivano in tempi differenti
