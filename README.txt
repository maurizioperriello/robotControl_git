CHANGELOG:
2.1:
	-Aggiornamento a "controlNode_billAvoier.py"

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
