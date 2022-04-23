FILE:
-"simplerSearcher" per la ricerca del target;
-"billAvoider" per implementare l'allontanamento dall'operatore;
-"completeController" per unificare i due agenti;
-La cartella "other_scripts" contiene file non più utilizzati.


CHANGELOG:
5.0:
	-Si modificano le scene di CoppeliaSim e si realizzano i file per la simulazione definitiva

4.0:
	-Si cambia il modello del robot (si passa a ur10e definitivo) e si modificano gli ambienti in linea con quelli reali
	-Si modificano di conseguenza gli scripts dei due agenti (marginalmente)

3.2.1:
	-Aggiornamento di "..._billAvoider.py"
	-Aggiornamento di "..._simplerSearcher.py": ora si salva lo score su un pandas dataframe

3.2:
	-Aggiornamento di "..._billAvoider.py"
	-Aggiornamenti marginali a "DDPG_classes.py" (ora si può evitare di caricare la memoria sulla base dell'evaluate) e, di conseguenza, a "..._simplerSearcher.py"

3.1:
	-Aggiunta scena di CoppeliaSim "ur10e_completeEnv.ttt"	

3.0:
	-"Simpler Searcher" definitive edition

2.3.4:
	-modifiche a "controlNode_billAvoider.py": si modifica il reward in modo tale da inserire una componente positiva

2.3.3:
	-aggiunta files CoppeliaSim

2.3.2:
	-aggiornamento a "controlNode_billAvoider.py"

2.3.1:
	-Riordinamento repository

2.3:
	-aggiornamento a "controlNode_billAvoider.py"

2.2.1:
	-Piccole modifiche funzionali a "completeController.py"

2.2:
	-Prima bozza di "completeController.py"

2.1:
	-Aggiornamento a "controlNode_billAvoider.py"

2.0.2:
	-Aggiornamento marginale a "ddpg_class.py": ora non si salva più la memoria alla fine della prima iterazione

2.0.1:
	-Aggiornamento a "listener_class.py" per far fronte alla nuova scena col nuovo Bill

2.0:
	-Aggiunto "controlNodes_simplerSearcher.py", aggiunto l'ambiente anaconda per fare funzionare i vari files, modifiche minori

1.10.2:
	-piccola correzione a "listener_class.py"

1.10.1:
	-Modifica a "listener_class.py" per includere il Bill controllor e aggiornamento del targetHunter

1.10:
	-Modifica di "Bill_controller.py" e creazione di "controlNode_BillAvoider.py"

1.9.1:
	-Modifica file "ddpg_classes.py" per salvare correttamente l'indice della memoria

9th COMMIT (v1.9):
	-Modifica del file "controlNodes_targetHunter.py", aggiunta di altri rewards

8th COMMIT:
	-Correzione del file "listener.py"

7th COMMIT:
	-Aggiunto file "bill_controller.py"

6th COMMIT:
	-Aggiornamento del file "controlNodes_targetHunter.py": aggiunta la possibilità di selezionare le iterazioni da salvare in memoria,
	 modificato il reward (due termini, uno per posizione e velocità e uno per l'orientamento)
