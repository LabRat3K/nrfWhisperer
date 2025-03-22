#ifndef _lcd_menu_h
#define _lcd_menu_h

#define MENU_TYPE_MENU   0x00
#define MENU_TYPE_SELECT 0x01
#define MENU_TYPE_ACTION 0x02

typedef int (*menuHdlr)(int key);

// -----  Select/Option menu type -----
typedef int (*defaultHdlr)(bool display);// Return index of current setting

typedef struct sSelectItem {
	char option[17];
	char value;
} tSelectItem;

typedef struct sSelectMenu {
	defaultHdlr pGetDefault; // Display Default & return index
	uint8_t * (selectVar);      // Variable to be updated by this handler
	tSelectItem *list;	 // Pointer Itemto the Select/Option list
} tSelectMenu;

// -----  Menu type -----
typedef struct sMenuItem {
  char          title[17];
  uint8_t 	menuType;
  menuHdlr      pMenuAction;
  union {
     sMenuItem     *pSubMenu;
     tSelectMenu   *pSelectMenu;
  };
  uint8_t       menuSize;        // Number of elements in the list
} tMenuItem;

#endif

