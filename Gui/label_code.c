#include "label_code.h"
#include "guix.h"


const u8* MAINMENU_ICONNAME[GUI_LANGUAGE_NUM][4]=
{
	{
		"装卸耗材","SD打印","应用","系统",
	},
	{
		"裝卸耗材","SD打印","應用","系統",
	},
	{
		"UNLOAD","SD_PRINT","APPCATION","SYSTEM",
	},
	//韩语
	{
		"鞘扼膏飘 背眉","橇赴飘","绢敲府纳捞记","矫胶袍",
	},
	//西班牙语
	{
		"Calienta.","Impresión","Aplicación","Sistema",
	},
	//德语
	{
		"ENTLADEN","AUSDRUCKEN","ANWENDUNG","SYSTEM",
	},
	//俄语
	{
		"Грузить","Печать","Пункт","Систем",
	},
	//法语
	{
		"Décharger","Imprimer","Application","Système",
	},
	//葡萄牙语
	{
		"Aquecimento","Impresso","Aplique isso","Sistema",
	},
	//意大利语
	{
		"SCARICO","STAMPARE","APPLICAZIONE","SISTEMA",
	},
	
};



const u8* HEAT_ICONNAME[GUI_LANGUAGE_NUM][6]=
{
		{
			"打印头预热","热床预热","打印头2预热","--:当前温度","--:目标温度","℃",
		},
		{
			"打印頭預熱","熱床預熱","打印頭2預熱","--:當前溫度","--:目標溫度","℃",
		},
		
		{
			"HEAD PREHEAT","BED PREHEAT","HEAD2 PREHEAT","--:CURRENT","--:TARGET","℃",
		},
		{
			"抗凯1","海靛 抗凯","抗凯2","--:泅犁 柯档","--:格钎 柯档","档",
		},
		{
			"Calienta 1","Plataforma.","Calienta 2","--:Corriente","--:Objetivo","℃",
		},
		{
			"Heizung 1","Heizbett","Heizung 2","--:Aktuell","--:Zeil","℃",
		},
		{
			"обогрев1","Кровать","обогрев2","--:Текущая","--:Целевая","℃",
		},
		{
			"Préchauffage1","Lit chaud","Préchauffage2","--:Actuelle","--:Cible","℃",
		},
		{
			"Aquecimento1","Cama quente","Aquecimento2","--:Corrente","--:Alvo","℃",
		},
		{
			"Riscalda.1","Letto caldo","Riscalda.2","--:ATTUALE","--:BERSAGLIO","℃",
		},
};


//系统设置主目录表
const u8* SYSTEM_MENU[GUI_LANGUAGE_NUM][4]=//系统一级目录的个数      数组里的元素都是字符串的首地址
{
	{
	"关于","返回","中文","恢复出厂设置",
	},
	{
	"關於","返回","中文（繁體）","恢復出廠設置",
	},
	{
	"ABOUT","BACK","English","FACTORY DEFAULT",
	},
	{
	"眠啊 沥焊","第肺啊扁","茄惫绢", "傍厘 檬扁拳"
	},
	{
	"Sobre","Regresar","Italiano","Inicialización.",
	},
	{
	"über","Zurück","Deutsch","Werkseinstellung",
	},
	{
	"Справка","Назад","Pусский язык","начальный",
	},
	{
	"à propos de","Retour","Fran?ais","Initialisation",
	},
	{
	"Sobre","Volte","- Português","Recupere - se",
	},
	{
	"Informazioni","Indietro","Italiano","Inizializzare il",
	},
};



//系统设置主目录表<=12char
const u8* TOOLS_MENU[GUI_LANGUAGE_NUM][7]=//系统一级目录的个数      数组里的元素都是字符串的首地址
{
	{
	 "热床预热","手动","调平","WIFI","更新主题","返回","更多",
	},
	{
	 "熱床預熱","手動","調平","WIFI","更新主題","返回","更多",
	},
	{
	 "BED PREHEAT","MANUAL","LEVELING","WIFI","UPDATE_THEME","BACK","MORE",
	},
	{
	 "海靛 抗凯","谅钎 捞悼","饭骇傅","WIFI","抛付诀单捞飘","第肺啊扁","磊技洒",
	},
	{
	 "Plataforma.","Manual","Nivelación","WIFI","Actualizar","Regresar","Más.",
	},
	{
	 "Heizbett","Manuell","Nivellierung","WIFI","Aktualisieren","Zurück","Mehr",
	},
	{
	 "Кровать","Ручной","Регулировать","WIFI","Обновлять","Назад","больше",
	},
	{
	 "Lit chaud","Manuel","Niveler","WIFI","Remplacer","Retour","Plus encore",
	},
	{
	 "Cama quente","Manual","Nivelamento","WIFI","Atualizar","Volte","- mais",
	},
	{
	 "Letto caldo","Manuale","Livellamento","WIFI","Aggiornamento","Ritorno","Di più",
	},
	
};


const u8* POWER_OFF_LABE[GUI_LANGUAGE_NUM][2]=//系统一级目录的个数      数组里的元素都是字符串的首地址
{

	{
	 "主电源已断开!!!","请接通电源"
	},
	{
	 "主電源已斷開!!!","請接通電源"
	},
	{
	 "POWER DISCONNECT!!!","PLEASE CONNECT POWER"
	},
	{
	 "傈盔捞 波脸促!","傈盔阑 难绞矫坷."
	},
	{
	 "Está apagado!","Por favor, encienda."
	},
	{
	 "Stromausfall !!!","Bitte einschalten."
	},
	{
	 "питание отключено!","включите питание"
	},
	{
	 "Coupure de courant!!!","Veuillez brancher l’alimentation"
	},
	{
	 "Falta de Energia!!!","Por favor liquei a fonte de Energia"
	},
	{
	 "L'ALIMENTAZIONE PRINCIPALE E’STATO SCOLLEGATA!","SI PREGA DI ACCENDERE L’ALIMENTAZIONE."
	},
};
const u8* CONTINUE_PRINT_LABE[GUI_LANGUAGE_NUM][3]=//系统一级目录的个数      数组里的元素都是字符串的首地址
{

	{
	 "发现有未完成任务","继续完成打印?","确定打印?"
	},
	{
	 "發現有未完成任務","繼續完成打印?","確定打印?",
	},
	{
	 "FIND UNFINISHED TASK","CONTIUNE TO COMPLETE PRINT?","Are you sure to print?"
	},
	{
	 "牢尖登瘤 臼篮 颇老 惯斑.","拌加窍矫摆嚼聪鳖?","免仿 矫累?"
	},
	{
	 "Hay una tarea inacabada,","continúe imprimiendo?","estás seguro?"
	},
	{
	 "unfertige Aufgabe finden,","weiter zu drücken?","Sind Sie sicher?"
	},
	{
	 "Не завершено,","продолжай?","Вы уверены?"
	},
	{
	 "Une tache non finie,","continuez à accomplir l’impression?","Voulez-vous vraiment imprimer?"
	},
	{
	 "Encontrar tarefas inacabadas,","Quer continuar?","Tem certeza de que deseja imprimir?"
	},
	{
	 "TROVI UN’ATTIVITA’ INCOMPLETA","E CONTINUI A FINIRE LA STAMPA?","SEI SICURA?"
	},
	
};

const u8* MATERIAL_OVER_LABE[GUI_LANGUAGE_NUM][2]=//系统一级目录的个数      数组里的元素都是字符串的首地址
{

	{
	 "耗材已用完!!!","请更换打印耗材"
	},
	{
	 "耗材已用完!!!","請更換打印耗材"
	},
	{
	 "MATERIAL USED UP!!!","PLEASE REPLACE NEW MATERIAL"
	},
	{
	 "家葛前 荤侩!","家葛前阑 背眉窍绞矫坷."
	},
	{
	 "Materiales consumibles!","Reemplace El material!"
	},
	{
	 "Material leer !!!","Material ersetzen."
	},
	{
	 "исчерпан!","заменить На!"
	},
	{
	 "Matériel déjà épuisé!","Veuillez le remplacer."
	},
	{
	 "Os materiais foram usados!","Substitua o material."
	},
	{
	 "I CONSUMABILI SONO ESAURITI!","SI PREGA DI SOSTITUIRLO."
	},
	
};

const u8* MATERIAL_OK_LABE[GUI_LANGUAGE_NUM][2]=//系统一级目录的个数      数组里的元素都是字符串的首地址
{

	{
	 "耗材更换完毕","继续打印?"
	},
	{
	 "耗材更換完畢","繼續打印?"
	},
	{
	 "MATERIAL IS OK!!!","CONTIUNE TO PRINT?"
	},
	{
	 "背眉 肯丰!","牢尖 拌加?"
	},
	{
	 "Complete El reemplazo","Continúe imprimiendo?"
	},
	{
	 "Material ist ok !!!","weiter zu drücken?"
	},
	{
	 "Был заменен!","- продолжай?"
	},
	{
	 "Terminer le remplacement!","Continuez à imprimer?"
	},
	{
	 "Processo concluído!","Continuar para imprir?"
	},
	{
	 "IL MATERIALE è OK !","CONTINUA A STAMPARE?"
	},
};

 const u8* CONFIRM_CANCEL_LABE[GUI_LANGUAGE_NUM][2]=//系统一级目录的个数      数组里的元素都是字符串的首地址
{

	{
	 "终止任务？", "打印耗时:",
	},
	{
	 "終止任務？","打印耗時:",
	},
	{
	 "CANCEL TASK?","Time consuming:",
	},
	{
	 "牢尖 吝瘤?","牢尖 矫埃:",
	},
	{
	 "Cancela la tarea?","Tiempo de impresión:",
	},
	{
	 "Aufgabe abbrechen?","Zeit zum Drucken:",
	},
	{
	 "Вы закончили?","время вышло:",
	},
	{
	 "Supprimez la tache?","Temps nécessaire:",
	},
	{
	 "Cancelar a tarefa?","Demora muito tempo:",
	},
	{
	 "ANNULLA COMPITO?","RICHIEDE TEMPO:",
	},
};


const u8* ERR_COLD_MENU[GUI_LANGUAGE_NUM][2]=//系统一级目录的个数      数组里的元素都是字符串的首地址
{
	{
	 "未达到挤出温度","请先预热打印头",
	},
	{
	 "未逹到擠出溫度","請先預熱打印頭",
	},
	{
	 "ERR COLD EXTRUDE","PLEASE PREHEATING FIRST",
	},
	{
	 "柯档啊 呈公 撤促","刚历 啊凯窍绞矫坷."
	},
	{
	 "Temperatura es muy Baja,","Precalentar primero.",
	},
	{
	 "Fehler kaltes Extrudieren,","die Temperatur erh?hen",
	},
	{
	 "температура низкая,","сначала нагрейте",
	},
	{
	 "Température est trop basse,","Préchauffez d'abord",
	},
	{
	 "Temperatura está baixa,","Aumente a temperatura",
	},
	{
	 "ESTRUDI FREDDO ERR,","PRIMA DI PRERISCALDARE PRIMA",
	},
};

 const u8* LANGUAGE_MENU[GUI_LANGUAGE_NUM][GUI_LANGUAGE_NUM] =
 {
	 {"中文","中文(繁体)","英语","韩语","西班牙语","德语","俄语","法语","葡萄牙语","意大利语",
	 },
	 {"中文","中文(繁體)","英語","韓語","西班牙語","德語","俄語","法語","葡萄牙語","意大利語",
	 },
	 {"Chinese","Traditional Chinese","English","Korean","Spanish","German","Russian","French","Portuguese","Italian",
	 },
	 {"吝惫绢","吝惫绢 锅眉","康绢","茄惫绢","胶其牢绢","刀老绢","矾矫酒绢","橇尔胶绢","器福捧哎绢","捞呕府酒绢",
	 },
	 {"Chino.","Chino tradicional","Inglés.","Coreano.","Espagnol","Alemán.","Ruso.","Francés.","Portugués.","Italiano.",
	 },
	 {"Chinesisch","Traditionelles Chinesisch","Englisch","Koreanisch","Spanisch","Deutsch","Russisch","French","Portugiesisch","Italienisch",
	 },
	 {"Chinese","Traditional Chinese","English","Korean","Spanish","German","Russian","French","Portuguese","Italian",
	 },
	 {"Chinois","Chinois traditionnel","En anglais.","Coréen","Espagnol","Allemand","Russe","French","Portugal.","En italien",
	 },
	 {"Em chinês","Chinês tradicional","Em inglês","Coreano","Espanhol","German","Em Russo","Francês","Português","Italiano",
	 },
	 {"In cinese","Cinese tradizionale","In inglese","Coreano","Spagnolo","In tedesco","Russo","Il francese","Portoghese","In italiano",
	 },
 };





