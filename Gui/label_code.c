#include "label_code.h"
#include "guix.h"


const u8* MAINMENU_ICONNAME[GUI_LANGUAGE_NUM][4]=
{
	{
		"װж�Ĳ�","SD��ӡ","Ӧ��","ϵͳ",
	},
	{
		"�bж�Ĳ�","SD��ӡ","����","ϵ�y",
	},
	{
		"UNLOAD","SD_PRINT","APPCATION","SYSTEM",
	},
	//����
	{
		"�ʶ��Ʈ ��ü","����Ʈ","���ø����̼�","�ý���",
	},
	//��������
	{
		"Calienta.","Impresi��n","Aplicaci��n","Sistema",
	},
	//����
	{
		"ENTLADEN","AUSDRUCKEN","ANWENDUNG","SYSTEM",
	},
	//����
	{
		"�����٧ڧ��","���֧�ѧ��","����ߧܧ�","���ڧ��֧�",
	},
	//����
	{
		"D��charger","Imprimer","Application","Syst��me",
	},
	//��������
	{
		"Aquecimento","Impresso","Aplique isso","Sistema",
	},
	//�������
	{
		"SCARICO","STAMPARE","APPLICAZIONE","SISTEMA",
	},
	
};



const u8* HEAT_ICONNAME[GUI_LANGUAGE_NUM][6]=
{
		{
			"��ӡͷԤ��","�ȴ�Ԥ��","��ӡͷ2Ԥ��","--:��ǰ�¶�","--:Ŀ���¶�","��",
		},
		{
			"��ӡ�^�A��","�ᴲ�A��","��ӡ�^2�A��","--:��ǰ�ض�","--:Ŀ�˜ض�","��",
		},
		
		{
			"HEAD PREHEAT","BED PREHEAT","HEAD2 PREHEAT","--:CURRENT","--:TARGET","��",
		},
		{
			"����1","���� ����","����2","--:���� �µ�","--:��ǥ �µ�","��",
		},
		{
			"Calienta 1","Plataforma.","Calienta 2","--:Corriente","--:Objetivo","��",
		},
		{
			"Heizung 1","Heizbett","Heizung 2","--:Aktuell","--:Zeil","��",
		},
		{
			"��ҧ�ԧ�֧�1","�����ӧѧ��","��ҧ�ԧ�֧�2","--:���֧ܧ��ѧ�","--:���֧ݧ֧ӧѧ�","��",
		},
		{
			"Pr��chauffage1","Lit chaud","Pr��chauffage2","--:Actuelle","--:Cible","��",
		},
		{
			"Aquecimento1","Cama quente","Aquecimento2","--:Corrente","--:Alvo","��",
		},
		{
			"Riscalda.1","Letto caldo","Riscalda.2","--:ATTUALE","--:BERSAGLIO","��",
		},
};


//ϵͳ������Ŀ¼��
const u8* SYSTEM_MENU[GUI_LANGUAGE_NUM][4]=//ϵͳһ��Ŀ¼�ĸ���      �������Ԫ�ض����ַ������׵�ַ
{
	{
	"����","����","����","�ָ���������",
	},
	{
	"�P�","����","���ģ����w��","�֏ͳ��S�O��",
	},
	{
	"ABOUT","BACK","English","FACTORY DEFAULT",
	},
	{
	"�߰� ����","�ڷΰ���","�ѱ���", "���� �ʱ�ȭ"
	},
	{
	"Sobre","Regresar","Italiano","Inicializaci��n.",
	},
	{
	"��ber","Zur��ck","Deutsch","Werkseinstellung",
	},
	{
	"�����ѧӧܧ�","���ѧ٧ѧ�","P����ܧڧ� ��٧��","�ߧѧ�ѧݧ�ߧ��",
	},
	{
	"�� propos de","Retour","Fran?ais","Initialisation",
	},
	{
	"Sobre","Volte","- Portugu��s","Recupere - se",
	},
	{
	"Informazioni","Indietro","Italiano","Inizializzare il",
	},
};



//ϵͳ������Ŀ¼��<=12char
const u8* TOOLS_MENU[GUI_LANGUAGE_NUM][7]=//ϵͳһ��Ŀ¼�ĸ���      �������Ԫ�ض����ַ������׵�ַ
{
	{
	 "�ȴ�Ԥ��","�ֶ�","��ƽ","WIFI","��������","����","����",
	},
	{
	 "�ᴲ�A��","�ք�","�{ƽ","WIFI","�������}","����","����",
	},
	{
	 "BED PREHEAT","MANUAL","LEVELING","WIFI","UPDATE_THEME","BACK","MORE",
	},
	{
	 "���� ����","��ǥ �̵�","������","WIFI","�׸�������Ʈ","�ڷΰ���","�ڼ���",
	},
	{
	 "Plataforma.","Manual","Nivelaci��n","WIFI","Actualizar","Regresar","M��s.",
	},
	{
	 "Heizbett","Manuell","Nivellierung","WIFI","Aktualisieren","Zur��ck","Mehr",
	},
	{
	 "�����ӧѧ��","�����ߧ��","���֧ԧ�ݧڧ��ӧѧ��","WIFI","���ҧߧ�ӧݧ���","���ѧ٧ѧ�","�ҧ�ݧ���",
	},
	{
	 "Lit chaud","Manuel","Niveler","WIFI","Remplacer","Retour","Plus encore",
	},
	{
	 "Cama quente","Manual","Nivelamento","WIFI","Atualizar","Volte","- mais",
	},
	{
	 "Letto caldo","Manuale","Livellamento","WIFI","Aggiornamento","Ritorno","Di pi��",
	},
	
};


const u8* POWER_OFF_LABE[GUI_LANGUAGE_NUM][2]=//ϵͳһ��Ŀ¼�ĸ���      �������Ԫ�ض����ַ������׵�ַ
{

	{
	 "����Դ�ѶϿ�!!!","���ͨ��Դ"
	},
	{
	 "���Դ�є��_!!!","Ո��ͨ�Դ"
	},
	{
	 "POWER DISCONNECT!!!","PLEASE CONNECT POWER"
	},
	{
	 "������ ������!","������ �ѽʽÿ�."
	},
	{
	 "Est�� apagado!","Por favor, encienda."
	},
	{
	 "Stromausfall !!!","Bitte einschalten."
	},
	{
	 "��ڧ�ѧߧڧ� ���ܧݧ��֧ߧ�!","�ӧܧݧ��ڧ�� ��ڧ�ѧߧڧ�"
	},
	{
	 "Coupure de courant!!!","Veuillez brancher l��alimentation"
	},
	{
	 "Falta de Energia!!!","Por favor liquei a fonte de Energia"
	},
	{
	 "L'ALIMENTAZIONE PRINCIPALE E��STATO SCOLLEGATA!","SI PREGA DI ACCENDERE L��ALIMENTAZIONE."
	},
};
const u8* CONTINUE_PRINT_LABE[GUI_LANGUAGE_NUM][3]=//ϵͳһ��Ŀ¼�ĸ���      �������Ԫ�ض����ַ������׵�ַ
{

	{
	 "������δ�������","������ɴ�ӡ?","ȷ����ӡ?"
	},
	{
	 "�l�F��δ����΄�","�^�m��ɴ�ӡ?","�_����ӡ?",
	},
	{
	 "FIND UNFINISHED TASK","CONTIUNE TO COMPLETE PRINT?","Are you sure to print?"
	},
	{
	 "�μ���� ���� ���� �߰�.","����Ͻðڽ��ϱ�?","��� ����?"
	},
	{
	 "Hay una tarea inacabada,","contin��e imprimiendo?","est��s seguro?"
	},
	{
	 "unfertige Aufgabe finden,","weiter zu dr��cken?","Sind Sie sicher?"
	},
	{
	 "���� �٧ѧӧ֧��֧ߧ�,","����է�ݧاѧ�?","���� ��ӧ֧�֧ߧ�?"
	},
	{
	 "Une tache non finie,","continuez �� accomplir l��impression?","Voulez-vous vraiment imprimer?"
	},
	{
	 "Encontrar tarefas inacabadas,","Quer continuar?","Tem certeza de que deseja imprimir?"
	},
	{
	 "TROVI UN��ATTIVITA�� INCOMPLETA","E CONTINUI A FINIRE LA STAMPA?","SEI SICURA?"
	},
	
};

const u8* MATERIAL_OVER_LABE[GUI_LANGUAGE_NUM][2]=//ϵͳһ��Ŀ¼�ĸ���      �������Ԫ�ض����ַ������׵�ַ
{

	{
	 "�Ĳ�������!!!","�������ӡ�Ĳ�"
	},
	{
	 "�Ĳ�������!!!","Ո���Q��ӡ�Ĳ�"
	},
	{
	 "MATERIAL USED UP!!!","PLEASE REPLACE NEW MATERIAL"
	},
	{
	 "�Ҹ�ǰ ���!","�Ҹ�ǰ�� ��ü�Ͻʽÿ�."
	},
	{
	 "Materiales consumibles!","Reemplace El material!"
	},
	{
	 "Material leer !!!","Material ersetzen."
	},
	{
	 "�ڧ��֧��ѧ�!","�٧ѧާ֧ߧڧ�� ����!"
	},
	{
	 "Mat��riel d��j�� ��puis��!","Veuillez le remplacer."
	},
	{
	 "Os materiais foram usados!","Substitua o material."
	},
	{
	 "I CONSUMABILI SONO ESAURITI!","SI PREGA DI SOSTITUIRLO."
	},
	
};

const u8* MATERIAL_OK_LABE[GUI_LANGUAGE_NUM][2]=//ϵͳһ��Ŀ¼�ĸ���      �������Ԫ�ض����ַ������׵�ַ
{

	{
	 "�Ĳĸ������","������ӡ?"
	},
	{
	 "�Ĳĸ��Q�ꮅ","�^�m��ӡ?"
	},
	{
	 "MATERIAL IS OK!!!","CONTIUNE TO PRINT?"
	},
	{
	 "��ü �Ϸ�!","�μ� ���?"
	},
	{
	 "Complete El reemplazo","Contin��e imprimiendo?"
	},
	{
	 "Material ist ok !!!","weiter zu dr��cken?"
	},
	{
	 "����� �٧ѧާ֧ߧ֧�!","- ����է�ݧاѧ�?"
	},
	{
	 "Terminer le remplacement!","Continuez �� imprimer?"
	},
	{
	 "Processo conclu��do!","Continuar para imprir?"
	},
	{
	 "IL MATERIALE �� OK !","CONTINUA A STAMPARE?"
	},
};

 const u8* CONFIRM_CANCEL_LABE[GUI_LANGUAGE_NUM][2]=//ϵͳһ��Ŀ¼�ĸ���      �������Ԫ�ض����ַ������׵�ַ
{

	{
	 "��ֹ����", "��ӡ��ʱ:",
	},
	{
	 "�Kֹ�΄գ�","��ӡ�ĕr:",
	},
	{
	 "CANCEL TASK?","Time consuming:",
	},
	{
	 "�μ� ����?","�μ� �ð�:",
	},
	{
	 "Cancela la tarea?","Tiempo de impresi��n:",
	},
	{
	 "Aufgabe abbrechen?","Zeit zum Drucken:",
	},
	{
	 "���� �٧ѧܧ�ߧ�ڧݧ�?","�ӧ�֧ާ� �ӧ��ݧ�:",
	},
	{
	 "Supprimez la tache?","Temps n��cessaire:",
	},
	{
	 "Cancelar a tarefa?","Demora muito tempo:",
	},
	{
	 "ANNULLA COMPITO?","RICHIEDE TEMPO:",
	},
};


const u8* ERR_COLD_MENU[GUI_LANGUAGE_NUM][2]=//ϵͳһ��Ŀ¼�ĸ���      �������Ԫ�ض����ַ������׵�ַ
{
	{
	 "δ�ﵽ�����¶�","����Ԥ�ȴ�ӡͷ",
	},
	{
	 "δ�Q���D���ض�","Ո���A���ӡ�^",
	},
	{
	 "ERR COLD EXTRUDE","PLEASE PREHEATING FIRST",
	},
	{
	 "�µ��� �ʹ� ����","���� �����Ͻʽÿ�."
	},
	{
	 "Temperatura es muy Baja,","Precalentar primero.",
	},
	{
	 "Fehler kaltes Extrudieren,","die Temperatur erh?hen",
	},
	{
	 "��֧ާ�֧�ѧ���� �ߧڧ٧ܧѧ�,","��ߧѧ�ѧݧ� �ߧѧԧ�֧ۧ��",
	},
	{
	 "Temp��rature est trop basse,","Pr��chauffez d'abord",
	},
	{
	 "Temperatura est�� baixa,","Aumente a temperatura",
	},
	{
	 "ESTRUDI FREDDO ERR,","PRIMA DI PRERISCALDARE PRIMA",
	},
};

 const u8* LANGUAGE_MENU[GUI_LANGUAGE_NUM][GUI_LANGUAGE_NUM] =
 {
	 {"����","����(����)","Ӣ��","����","��������","����","����","����","��������","�������",
	 },
	 {"����","����(���w)","Ӣ�Z","�n�Z","�������Z","���Z","���Z","���Z","�������Z","������Z",
	 },
	 {"Chinese","Traditional Chinese","English","Korean","Spanish","German","Russian","French","Portuguese","Italian",
	 },
	 {"�߱���","�߱��� ��ü","����","�ѱ���","�����ξ�","���Ͼ�","���þƾ�","��������","����������","��Ż���ƾ�",
	 },
	 {"Chino.","Chino tradicional","Ingl��s.","Coreano.","Espagnol","Alem��n.","Ruso.","Franc��s.","Portugu��s.","Italiano.",
	 },
	 {"Chinesisch","Traditionelles Chinesisch","Englisch","Koreanisch","Spanisch","Deutsch","Russisch","French","Portugiesisch","Italienisch",
	 },
	 {"Chinese","Traditional Chinese","English","Korean","Spanish","German","Russian","French","Portuguese","Italian",
	 },
	 {"Chinois","Chinois traditionnel","En anglais.","Cor��en","Espagnol","Allemand","Russe","French","Portugal.","En italien",
	 },
	 {"Em chin��s","Chin��s tradicional","Em ingl��s","Coreano","Espanhol","German","Em Russo","Franc��s","Portugu��s","Italiano",
	 },
	 {"In cinese","Cinese tradizionale","In inglese","Coreano","Spagnolo","In tedesco","Russo","Il francese","Portoghese","In italiano",
	 },
 };





