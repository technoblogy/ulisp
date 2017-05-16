/* uLisp Version 1.9 - www.ulisp.com
   David Johnson-Davies - www.technoblogy.com - 16th May 2017

   Licensed under the MIT license: https://opensource.org/licenses/MIT
*/

#include <setjmp.h>
#include <SPI.h>
#include <avr/eeprom.h>

// Compile options

#define checkoverflow
// #define resetautorun
#define printfreespace
#define serialmonitor
// #define printgcs

// C Macros

#define nil                NULL
#define car(x)             (((object *) (x))->car)
#define cdr(x)             (((object *) (x))->cdr)

#define first(x)           (((object *) (x))->car)
#define second(x)          (car(cdr(x)))
#define cddr(x)            (cdr(cdr(x)))
#define third(x)           (car(cdr(cdr(x))))

#define push(x, y)         ((y) = cons((x),(y)))
#define pop(y)             ((y) = cdr(y))

#define numberp(x)         ((x)->type == NUMBER)
#define symbolp(x)         ((x)->type == SYMBOL)
#define stringp(x)         ((x)->type == STRING)
#define characterp(x)      ((x)->type == CHARACTER)
#define streamp(x)         ((x)->type == STREAM)

#define mark(x)            (car(x) = (object *)(((unsigned int)(car(x))) | MARKBIT))
#define unmark(x)          (car(x) = (object *)(((unsigned int)(car(x))) & ~MARKBIT))
#define marked(x)          ((((unsigned int)(car(x))) & MARKBIT) != 0)
#define MARKBIT            1

#define setflag(x)         (Flags = Flags | 1<<(x))
#define clrflag(x)         (Flags = Flags & ~(1<<(x)))
#define tstflag(x)         (Flags & 1<<(x))

// Constants

const int TRACEMAX = 3; // Number of traced functions
enum type { ZERO=0, SYMBOL=2, NUMBER=4, STREAM=6, CHARACTER=8, STRING=10, PAIR=12 };  // STRING and PAIR must be last
enum token { UNUSED, BRA, KET, QUO, DOT };
enum stream { SERIALSTREAM, I2CSTREAM, SPISTREAM };

enum function { SYMBOLS, NIL, TEE, NOTHING, AMPREST, LAMBDA, LET, LETSTAR, CLOSURE, SPECIAL_FORMS, QUOTE,
DEFUN, DEFVAR, SETQ, LOOP, PUSH, POP, INCF, DECF, SETF, DOLIST, DOTIMES, TRACE, UNTRACE, FORMILLIS,
WITHSERIAL, WITHI2C, WITHSPI, TAIL_FORMS, PROGN, RETURN, IF, COND, WHEN, UNLESS, AND, OR, FUNCTIONS, NOT,
NULLFN, CONS, ATOM, LISTP, CONSP, NUMBERP, SYMBOLP, STREAMP, EQ, CAR, FIRST, CDR, REST, CAAR, CADR,
SECOND, CDAR, CDDR, CAAAR, CAADR, CADAR, CADDR, THIRD, CDAAR, CDADR, CDDAR, CDDDR, LENGTH, LIST, REVERSE,
NTH, ASSOC, MEMBER, APPLY, FUNCALL, APPEND, MAPC, MAPCAR, ADD, SUBTRACT, MULTIPLY, DIVIDE, MOD, ONEPLUS,
ONEMINUS, ABS, RANDOM, MAX, MIN, NUMEQ, LESS, LESSEQ, GREATER, GREATEREQ, NOTEQ, PLUSP, MINUSP, ZEROP,
ODDP, EVENP, CHAR, CHARCODE, CODECHAR, CHARACTERP, STRINGP, STRINGEQ, STRINGLESS, STRINGGREATER, SORT,
STRINGFN, CONCATENATE, SUBSEQ, READFROMSTRING, PRINCTOSTRING, PRIN1TOSTRING, LOGAND, LOGIOR, LOGXOR,
LOGNOT, ASH, LOGBITP, EVAL, GLOBALS, LOCALS, MAKUNBOUND, BREAK, READ, PRIN1, PRINT, PRINC, TERPRI,
READBYTE, READLINE, WRITEBYTE, WRITESTRING, WRITELINE, RESTARTI2C, GC, ROOM, SAVEIMAGE, LOADIMAGE, CLS,
PINMODE, DIGITALREAD, DIGITALWRITE, ANALOGREAD, ANALOGWRITE, DELAY, MILLIS, NOTE, EDIT, PPRINT, ENDFUNCTIONS };

// Typedefs

typedef unsigned int symbol_t;

typedef struct sobject {
  union {
    struct {
      sobject *car;
      sobject *cdr;
    };
    struct {
      unsigned int type;
      union {
        symbol_t name;
        int integer;
      };
    };
  };
} object;

typedef object *(*fn_ptr_type)(object *, object *);

typedef struct {
  const char *string;
  fn_ptr_type fptr;
  int min;
  int max;
} tbl_entry_t;

typedef char (*gfun_t)();
typedef void (*pfun_t)(char);

// Workspace - sizes in bytes
#define WORDALIGNED __attribute__((aligned (2)))
#define BUFFERSIZE 18

#if defined(__AVR_ATmega328P__)
#define WORKSPACESIZE 316               /* Cells (4*bytes) */
#define IMAGEDATASIZE 254               /* Cells */
#define SYMBOLTABLESIZE BUFFERSIZE      /* Bytes - no long symbols */

#elif defined(__AVR_ATmega2560__)
#define WORKSPACESIZE 1216              /* Cells (4*bytes) */
#define IMAGEDATASIZE 893               /* Cells */
#define SYMBOLTABLESIZE 512             /* Bytes */

#elif defined(__AVR_ATmega1284P__)
#define WORKSPACESIZE 2816              /* Cells (4*bytes) */
#define IMAGEDATASIZE 893               /* Cells */
#define SYMBOLTABLESIZE 512             /* Bytes */

#endif

object Workspace[WORKSPACESIZE] WORDALIGNED;
char SymbolTable[SYMBOLTABLESIZE];

// Global variables

jmp_buf exception;
unsigned int Freespace = 0;
object *Freelist;
char *SymbolTop = SymbolTable;
extern uint8_t _end;
unsigned int I2CCount;
unsigned int TraceFn[TRACEMAX];
unsigned int TraceDepth[TRACEMAX];

object *GlobalEnv;
object *GCStack = NULL;
object *GlobalString;
int GlobalStringIndex = 0;
char BreakLevel = 0;
char LastChar = 0;
char LastPrint = 0;
char PrintReadably = 1;

// Flags
enum flag { RETURNFLAG, ESCAPE, EXITEDITOR };
volatile char Flags;

// Forward references
object *tee;
object *tf_progn (object *form, object *env);
object *eval (object *form, object *env);
object *read ();
void repl(object *env);
void printobject (object *form, pfun_t pfun);
char *lookupbuiltin (symbol_t name);
int lookupfn (symbol_t name);
int lookupmin(symbol_t name);
int lookupmax(symbol_t name);
int builtin (char* n);
void Display (char c);
void error (const __FlashStringHelper *);
void pfstring (const __FlashStringHelper *);
void pint (int i);
void pln ();
void pfl ();
void pchar (char c);
int gchar ();
object *apply (object *function, object *args, object **env);
char *lookupsymbol (symbol_t name);
void deletesymbol (symbol_t name);
object *edit(object *fun);
int subwidthlist (object *form, int w);
void superprint (object *form, int lm);
void supersub (object *form, int lm, int super);

// Set up workspace

void initworkspace () {
  Freelist = NULL;
  for (int i=WORKSPACESIZE-1; i>=0; i--) {
    object *obj = &Workspace[i];
    car(obj) = NULL;
    cdr(obj) = Freelist;
    Freelist = obj;
    Freespace++;
  }
}

object *myalloc () {
  if (Freespace == 0) error(PSTR("No room"));
  object *temp = Freelist;
  Freelist = cdr(Freelist);
  Freespace--;
  return temp;
}

inline void myfree (object *obj) {
  car(obj) = NULL;
  cdr(obj) = Freelist;
  Freelist = obj;
  Freespace++;
}

// Make each type of object

object *number (int n) {
  object *ptr = myalloc();
  ptr->type = NUMBER;
  ptr->integer = n;
  return ptr;
}

object *character (char c) {
  object *ptr = myalloc();
  ptr->type = CHARACTER;
  ptr->integer = c;
  return ptr;
}

object *cons (object *arg1, object *arg2) {
  object *ptr = myalloc();
  ptr->car = arg1;
  ptr->cdr = arg2;
  return ptr;
}

object *symbol (symbol_t name) {
  object *ptr = myalloc();
  ptr->type = SYMBOL;
  ptr->name = name;
  return ptr;
}

object *stream (unsigned char streamtype, unsigned char address) {
  object *ptr = myalloc();
  ptr->type = STREAM;
  ptr->integer = streamtype<<8 | address;
  return ptr;
}

// Garbage collection

void markobject (object *obj) {
  MARK:
  if (obj == NULL) return;
  if (marked(obj)) return;

  object* arg = car(obj);
  unsigned int type = obj->type;
  mark(obj);
  
  if (type >= PAIR || type == ZERO) { // cons
    markobject(arg);
    obj = cdr(obj);
    goto MARK;
  }

  if (type == STRING) {
    obj = cdr(obj);
    while (obj != NULL) {
      arg = car(obj);
      mark(obj);
      obj = arg;
    }
  }
}

void sweep () {
  Freelist = NULL;
  Freespace = 0;
  for (int i=WORKSPACESIZE-1; i>=0; i--) {
    object *obj = &Workspace[i];
    if (!marked(obj)) myfree(obj); else unmark(obj);
  }
}

void gc (object *form, object *env) {
  #if defined(printgcs)
  int start = Freespace;
  #endif
  markobject(tee); 
  markobject(GlobalEnv);
  markobject(GCStack);
  markobject(form);
  markobject(env);
  sweep();
  #if defined(printgcs)
  pserial('{'); pint(Freespace - start, pserial); pserial('}');
  #endif
}

// Compact image

void movepointer (object *from, object *to) {
  for (int i=0; i<WORKSPACESIZE; i++) {
    object *obj = &Workspace[i];
    unsigned int type = (obj->type) & ~MARKBIT;
    if (marked(obj) && (type >= STRING || type==ZERO)) {
      if (car(obj) == (object *)((unsigned int)from | MARKBIT)) 
        car(obj) = (object *)((unsigned int)to | MARKBIT);
      if (cdr(obj) == from) cdr(obj) = to;
    }
  }
  // Fix strings
  for (int i=0; i<WORKSPACESIZE; i++) {
    object *obj = &Workspace[i];
    if (marked(obj) && ((obj->type) & ~MARKBIT) == STRING) {
      obj = cdr(obj);
      while (obj != NULL) {
        if (cdr(obj) == to) cdr(obj) = from;
        obj = (object *)((unsigned int)(car(obj)) & ~MARKBIT);
      }
    }
  }
}
  
int compactimage (object **arg) {
  markobject(tee);
  markobject(GlobalEnv);
  markobject(GCStack);
  object *firstfree = Workspace;
  while (marked(firstfree)) firstfree++;
  object *obj = &Workspace[WORKSPACESIZE-1];
  while (firstfree < obj) {
    if (marked(obj)) {
      car(firstfree) = car(obj);
      cdr(firstfree) = cdr(obj);
      unmark(obj);
      movepointer(obj, firstfree);
      if (GlobalEnv == obj) GlobalEnv = firstfree;
      if (GCStack == obj) GCStack = firstfree;
      if (*arg == obj) *arg = firstfree;
      while (marked(firstfree)) firstfree++;
    }
    obj--;
  }
  sweep();
  return firstfree - Workspace;
}

// Save-image and load-image

typedef struct {
  unsigned int eval;
  unsigned int datasize;
  unsigned int globalenv;
  unsigned int gcstack;
  #if SYMBOLTABLESIZE > BUFFERSIZE
  unsigned int symboltop;
  char table[SYMBOLTABLESIZE];
  #endif
  object data[IMAGEDATASIZE/4];
} struct_image;

struct_image EEMEM image;

int saveimage (object *arg) {
  unsigned int imagesize = compactimage(&arg);
  // Save to EEPROM
  if (imagesize > IMAGEDATASIZE) {
    pfstring(PSTR("Error: Image size too large: "), pserial);
    pint(imagesize, pserial); pln(pserial);
    GCStack = NULL;
    longjmp(exception, 1);
  }
  eeprom_update_word(&image.datasize, imagesize);
  eeprom_update_word(&image.eval, (unsigned int)arg);
  eeprom_update_word(&image.globalenv, (unsigned int)GlobalEnv);
  eeprom_update_word(&image.gcstack, (unsigned int)GCStack);
  #if SYMBOLTABLESIZE > BUFFERSIZE
  eeprom_update_word(&image.symboltop, (unsigned int)SymbolTop);
  eeprom_update_block(SymbolTable, image.table, SYMBOLTABLESIZE);
  #endif
  eeprom_update_block(Workspace, image.data, imagesize*4);
  return imagesize;
}

int loadimage () {
  unsigned int imagesize = eeprom_read_word(&image.datasize);
  if (imagesize == 0 || imagesize == 0xFFFF) error(PSTR("No saved image"));
  GlobalEnv = (object *)eeprom_read_word(&image.globalenv);
  GCStack = (object *)eeprom_read_word(&image.gcstack);
  #if SYMBOLTABLESIZE > BUFFERSIZE
  SymbolTop = (char *)eeprom_read_word(&image.symboltop);
  eeprom_read_block(SymbolTable, image.table, SYMBOLTABLESIZE);
  #endif
  eeprom_read_block(Workspace, image.data, imagesize*4);
  gc(NULL, NULL);
  return imagesize;
}

void autorunimage () {
  object *nullenv = NULL;
  object *autorun = (object *)eeprom_read_word(&image.eval);
  if (autorun != NULL && (unsigned int)autorun != 0xFFFF) {
    loadimage();
    apply(autorun, NULL, &nullenv);
  }
}

// Error handling

void error (PGM_P string) {
  pfl(pserial); pfstring(PSTR("Error: "), pserial);
  pfstring(string, pserial); pln(pserial);
  GCStack = NULL;
  longjmp(exception, 1);
}

void error2 (object *symbol, PGM_P string) {
  pfl(pserial); pfstring(PSTR("Error: "), pserial);
  if (symbol == NULL) pfstring(PSTR("function "), pserial);
  else { pserial('\''); printobject(symbol, pserial); pfstring(PSTR("' "), pserial); }
  pfstring(string, pserial); pln(pserial);
  GCStack = NULL;
  longjmp(exception, 1);
}

// Tracing

boolean tracing (symbol_t name) {
  if (name == 0) return 0;
  int i = 0;
  while (i < TRACEMAX) {
    if (TraceFn[i] == name) return i+1;
    i++;
  }
  return 0;
}

void trace (symbol_t name) {
  if (tracing(name)) error(PSTR("Already being traced"));
  int i = 0;
  while (i < TRACEMAX) {
    if (TraceFn[i] == 0) { TraceFn[i] = name; TraceDepth[i] = 0; return; }
    i++;
  }
  error(PSTR("Already tracing 3 functions"));
}

void untrace (symbol_t name) {
  int i = 0;
  while (i < TRACEMAX) {
    if (TraceFn[i] == name) { TraceFn[i] = 0; return; }
    i++;
  }
  error(PSTR("It wasn't being traced"));
}

// Helper functions

boolean consp (object *x) {
  if (x == NULL) return false;
  unsigned int type = x->type;
  return type >= PAIR || type == ZERO;
}

boolean atom (object *x) {
  if (x == NULL) return true;
  unsigned int type = x->type;
  return type < PAIR && type != ZERO;
}

boolean listp (object *x) {
  if (x == NULL) return true;
  unsigned int type = x->type;
  return type >= PAIR || type == ZERO;
}

int toradix40 (char ch) {
  if (ch == 0) return 0;
  if (ch >= '0' && ch <= '9') return ch-'0'+30;
  ch = ch | 0x20;
  if (ch >= 'a' && ch <= 'z') return ch-'a'+1;
  return -1; // Invalid
}

int fromradix40 (int n) {
  if (n >= 1 && n <= 26) return 'a'+n-1;
  if (n >= 30 && n <= 39) return '0'+n-30;
  return 0;
}

int pack40 (char *buffer) {
  return (((toradix40(buffer[0]) * 40) + toradix40(buffer[1])) * 40 + toradix40(buffer[2]));
}

boolean valid40 (char *buffer) {
 return (toradix40(buffer[0]) >= 0 && toradix40(buffer[1]) >= 0 && toradix40(buffer[2]) >= 0);
}

int digitvalue (char d) {
  if (d>='0' && d<='9') return d-'0';
  d = d | 0x20;
  if (d>='a' && d<='f') return d-'a'+10;
  return 16;
}

char *name (object *obj){
  if (obj->type != SYMBOL) error(PSTR("Error in name"));
  symbol_t x = obj->name;
  if (x < ENDFUNCTIONS) return lookupbuiltin(x);
  else if (x >= 64000) return lookupsymbol(x);
  char *buffer = SymbolTop;
  buffer[3] = '\0';
  for (int n=2; n>=0; n--) {
    buffer[n] = fromradix40(x % 40);
    x = x / 40;
  }
  return buffer;
}

int integer (object *obj){
  if (!numberp(obj)) error(PSTR("Not a number"));
  return obj->integer;
}

int fromchar (object *obj){
  if (!characterp(obj)) error(PSTR("Not a character"));
  return obj->integer;
}

int istream (object *obj){
  if (!streamp(obj)) error(PSTR("Not a stream"));
  return obj->integer;
}

int issymbol (object *obj, symbol_t n) {
  return symbolp(obj) && obj->name == n;
}

int eq (object *arg1, object *arg2) {
  int same_object = (arg1 == arg2);
  int same_value = (arg1->cdr == arg2->cdr);
  int same_symbol = (symbolp(arg1) && symbolp(arg2) && same_value);
  int same_number = (numberp(arg1) && numberp(arg2) && same_value);
  int same_character = (characterp(arg1) && characterp(arg2) && same_value);
  return same_object || same_symbol || same_number || same_character;
}

int listlength (object *list) {
  int length = 0;
  while (list != NULL) {
    list = cdr(list);
    length++;
  }
  return length;
}

// Association lists

object *assoc (object *key, object *list) {
  while (list != NULL) {
    object *pair = first(list);
    if (eq(key,car(pair))) return pair;
    list = cdr(list);
  }
  return nil;
}

object *delassoc (object *key, object **alist) {
  object *list = *alist;
  object *prev = NULL;
  while (list != NULL) {
    object *pair = first(list);
    if (eq(key,car(pair))) {
      if (prev == NULL) *alist = cdr(list);
      else cdr(prev) = cdr(list);
      return key;
    }
    prev = list;
    list = cdr(list);
  }
  return nil;
}

// String utilities

void indent (int spaces) {
  for (int i=0; i<spaces; i++) pserial(' ');
}

void buildstring (char ch, int *chars, object **head) {
  static object* tail;
  if (*chars == 0) {
    *chars = ch<<8;
    object *cell = myalloc();
    if (*head == NULL) *head = cell; else tail->car = cell;
    cell->car = NULL;
    cell->integer = *chars;
    tail = cell;
  } else {
    *chars = *chars | ch;
    tail->integer = *chars;
    *chars = 0;
  }
}

object *readstring (char delim, gfun_t gfun) {
  object *obj = myalloc();
  obj->type = STRING;
  char ch = gfun();
  object *head = NULL;
  int chars = 0;
  while (ch != delim) {
    if (ch == '\\') ch = gfun();
    buildstring(ch, &chars, &head);
    ch = gfun();
  }
  obj->cdr = head;
  return obj;
}

int stringlength (object *form) {
  int length = 0;
  form = cdr(form);
  while (form != NULL) {
    int chars = form->integer;
    if (chars & 0xFF) length++;
    if (chars & 0xFF00) length++;
    form = car(form);
  }
  return length;
}

char nthchar (object *string, int n) {
  object *arg = cdr(string);
  int top = n>>1;
  for (int i=0; i<top; i++) {
    if (arg == NULL) return 0;
    arg = car(arg);
  }
  if (arg == NULL) return 0;
  char ch;
  if (n&1) ch = (arg->integer) & 0xFF; else ch = (arg->integer)>>8 & 0xFF;
  return ch;
}

// Lookup variable in environment

object *value (symbol_t n, object *env) {
  while (env != NULL) {
    object *pair = car(env);
    if (pair != NULL && car(pair)->name == n) return pair;
    env = cdr(env);
  }
  return nil;
}

object *findvalue (object *var, object *env) {
  symbol_t varname = var->name;
  object *pair = value(varname, env);
  if (pair == NULL) pair = value(varname, GlobalEnv);
  if (pair == NULL) error2(var,PSTR("unknown variable"));
  return pair;
}

object *findtwin (object *var, object *env) {
  while (env != NULL) {
    object *pair = car(env);
    if (pair != NULL && car(pair) == var) return pair;
    env = cdr(env);
  }
  return NULL;
}

void dropframe (int tc, object **env) {
  if (tc) {
    while (*env != NULL && car(*env) != NULL) {
      pop(*env);
    }
  } else {
    push(nil, *env);
  }
}

// Handling closures
  
object *closure (object *fname, object *state, object *function, object *args, object **env) {
  int trace = tracing(fname->name);
  if (trace) {
    indent(TraceDepth[trace-1]<<1);
    pint(TraceDepth[trace-1]++, pserial);
    pserial(':'); pserial(' '); pserial('('); printobject(fname, pserial);
  }
  object *params = first(function);
  function = cdr(function);
  // Push state if not already in env
  while (state != NULL) {
    object *pair = first(state);
    if (findtwin(car(pair), *env) == NULL) push(pair, *env);
    state = cdr(state);
  }
  // Add arguments to environment
  while (params != NULL && args != NULL) {
    object *value;
    object *var = first(params);
    if (var->name == AMPREST) {
      params = cdr(params);
      var = first(params);
      value = args;
      args = NULL;
    } else {
      value = first(args);
      args = cdr(args);
    }
    push(cons(var,value), *env);
    params = cdr(params);
    if (trace) { pserial(' '); printobject(value, pserial); }
  }
  if (params != NULL) error2(fname, PSTR("has too few parameters"));
  if (args != NULL) error2(fname, PSTR("has too many parameters"));
  if (trace) { pserial(')'); pln(pserial); }
  // Do an implicit progn
  return tf_progn(function, *env);
}

object *apply (object *function, object *args, object **env) {
  if (symbolp(function)) {
    symbol_t name = function->name;
    int nargs = listlength(args);
    if (name >= ENDFUNCTIONS) error2(function, PSTR("is not valid here"));
    if (nargs<lookupmin(name)) error2(function, PSTR("has too few arguments"));
    if (nargs>lookupmax(name)) error2(function, PSTR("has too many arguments"));
    return ((fn_ptr_type)lookupfn(name))(args, *env);
  }
  if (listp(function) && issymbol(car(function), LAMBDA)) {
    function = cdr(function);
    object *result = closure(NULL, NULL, function, args, env);
    return eval(result, *env);
  }
  if (listp(function) && issymbol(car(function), CLOSURE)) {
    function = cdr(function);
    object *result = closure(NULL, car(function), cdr(function), args, env);
    return eval(result, *env);
  }
  error2(function, PSTR("is an illegal function"));
  return NULL;
}

// In-place operations

object **place (object *args, object *env) {
  if (atom(args)) return &cdr(findvalue(args, env));
  object* function = first(args);
  if (issymbol(function, CAR) || issymbol(function, FIRST)) {
    object *value = eval(second(args), env);
    if (!listp(value)) error(PSTR("Can't take car"));
    return &car(value);
  }
  if (issymbol(function, CDR) || issymbol(function, REST)) {
    object *value = eval(second(args), env);
    if (!listp(value)) error(PSTR("Can't take cdr"));
    return &cdr(value);
  }
  if (issymbol(function, NTH)) {
    int index = integer(eval(second(args), env));
    object *list = eval(third(args), env);
    if (atom(list)) error(PSTR("'nth' second argument is not a list"));
    while (index > 0) {
      list = cdr(list);
      if (list == NULL) error(PSTR("'nth' index out of range"));
      index--;
    }
    return &car(list);
  }
  error(PSTR("Illegal place"));
  return nil;
}

// Checked car and cdr

inline object *carx (object *arg) {
  if (!listp(arg)) error(PSTR("Can't take car"));
  if (arg == nil) return nil;
  return car(arg);
}

inline object *cdrx (object *arg) {
  if (!listp(arg)) error(PSTR("Can't take cdr"));
  if (arg == nil) return nil;
  return cdr(arg);
}

// I2C interface

#if defined(__AVR_ATmega328P__)
uint8_t const TWI_SDA_PIN = 18;
uint8_t const TWI_SCL_PIN = 19;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
uint8_t const TWI_SDA_PIN = 20;
uint8_t const TWI_SCL_PIN = 21;
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
uint8_t const TWI_SDA_PIN = 17;
uint8_t const TWI_SCL_PIN = 16;
#elif defined(__AVR_ATmega32U4__)
uint8_t const TWI_SDA_PIN = 6;
uint8_t const TWI_SCL_PIN = 5;
#endif

uint32_t const F_TWI = 400000L;  // Hardware I2C clock in Hz
uint8_t const TWSR_MTX_DATA_ACK = 0x28;
uint8_t const TWSR_MTX_ADR_ACK = 0x18;
uint8_t const TWSR_MRX_ADR_ACK = 0x40;
uint8_t const TWSR_START = 0x08;
uint8_t const TWSR_REP_START = 0x10;
uint8_t const I2C_READ = 1;
uint8_t const I2C_WRITE = 0;

void I2Cinit(bool enablePullup) {
  TWSR = 0;                        // no prescaler
  TWBR = (F_CPU/F_TWI - 16)/2;     // set bit rate factor
  if (enablePullup) {
    digitalWrite(TWI_SDA_PIN, HIGH);
    digitalWrite(TWI_SCL_PIN, HIGH);
  }
}

uint8_t I2Cread() {
  if (I2CCount != 0) I2CCount--;
  TWCR = 1<<TWINT | 1<<TWEN | ((I2CCount == 0) ? 0 : (1<<TWEA));
  while (!(TWCR & 1<<TWINT));
  return TWDR;
}

bool I2Cwrite(uint8_t data) {
  TWDR = data;
  TWCR = 1<<TWINT | 1 << TWEN;
  while (!(TWCR & 1<<TWINT));
  return (TWSR & 0xF8) == TWSR_MTX_DATA_ACK;
}

bool I2Cstart(uint8_t address, uint8_t read) {
  uint8_t addressRW = address<<1 | read;
  TWCR = 1<<TWINT | 1<<TWSTA | 1<<TWEN;    // send START condition
  while (!(TWCR & 1<<TWINT));
  if ((TWSR & 0xF8) != TWSR_START && (TWSR & 0xF8) != TWSR_REP_START) return false;
  TWDR = addressRW;  // send device address and direction
  TWCR = 1<<TWINT | 1<<TWEN;
  while (!(TWCR & 1<<TWINT));
  if (addressRW & I2C_READ) return (TWSR & 0xF8) == TWSR_MRX_ADR_ACK;
  else return (TWSR & 0xF8) == TWSR_MTX_ADR_ACK;
}

bool I2Crestart(uint8_t address, uint8_t read) {
  return I2Cstart(address, read);
}

void I2Cstop(uint8_t read) {
  (void) read;
  TWCR = 1<<TWINT | 1<<TWEN | 1<<TWSTO;
  while (TWCR & 1<<TWSTO); // wait until stop and bus released
}

inline char spiread () { return SPI.transfer(0); }
#if defined(__AVR_ATmega1284P__)
inline char serial1read () { while (!Serial1.available()) testescape(); return Serial1.read(); }
#elif defined(__AVR_ATmega2560__)
inline char serial1read () { while (!Serial1.available()) testescape(); return Serial1.read(); }
inline char serial2read () { while (!Serial2.available()) testescape(); return Serial2.read(); }
inline char serial3read () { while (!Serial3.available()) testescape(); return Serial3.read(); }
#endif

gfun_t gstreamfun (object *args) {
  int streamtype = SERIALSTREAM;
  int address = 0;
  gfun_t gfun = gserial;
  if (args != NULL) {
    int stream = istream(first(args));
    streamtype = stream>>8; address = stream & 0xFF;
  }
  if (streamtype == I2CSTREAM) gfun = (gfun_t)I2Cread;
  else if (streamtype == SPISTREAM) gfun = spiread;
  else if (streamtype == SERIALSTREAM) {
    if (address == 0) gfun = gserial;
    #if defined(__AVR_ATmega1284P__)
    else if (address == 1) gfun = serial1read;
    #elif defined(__AVR_ATmega2560__)
    else if (address == 1) gfun = serial1read;
    else if (address == 2) gfun = serial2read;
    else if (address == 3) gfun = serial3read;
    #endif
  }
  else error(PSTR("Unknown stream type"));
  return gfun;
}

inline void spiwrite (char c) { SPI.transfer(c); }
#if defined(__AVR_ATmega1284P__)
inline void serial1write (char c) { Serial1.write(c); }
#elif defined(__AVR_ATmega2560__)
inline void serial1write (char c) { Serial1.write(c); }
inline void serial2write (char c) { Serial2.write(c); }
inline void serial3write (char c) { Serial3.write(c); }
#endif

pfun_t pstreamfun (object *args) {
  int streamtype = SERIALSTREAM;
  int address = 0;
  pfun_t pfun = pserial;
  if (args != NULL) {
    int stream = istream(first(args));
    streamtype = stream>>8; address = stream & 0xFF;
  }
  if (streamtype == I2CSTREAM) pfun = (pfun_t)I2Cwrite;
  else if (streamtype == SPISTREAM) pfun = spiwrite;
  else if (streamtype == SERIALSTREAM) {
    if (address == 0) pfun = pserial;
    #if defined(__AVR_ATmega1284P__)
    else if (address == 1) pfun = serial1write;
    #elif defined(__AVR_ATmega2560__)
    else if (address == 1) pfun = serial1write;
    else if (address == 2) pfun = serial2write;
    else if (address == 3) pfun = serial3write;
    #endif
  }
  else error(PSTR("'write-string' unknown stream type"));
  return pfun;
}

// Check pins

void checkanalogread (int pin) {
#if defined(__AVR_ATmega328P__)
  if (!(pin>=0 && pin<=5)) error(PSTR("'analogread' invalid pin"));
#elif defined(__AVR_ATmega2560__)
  if (!(pin>=0 && pin<=15)) error(PSTR("'analogread' invalid pin"));
#elif defined(__AVR_ATmega1284P__)
  if (!(pin>=0 && pin<=7)) error(PSTR("'analogread' invalid pin"));
#endif
}

void checkanalogwrite (int pin) {
#if defined(__AVR_ATmega328P__)
  if (!(pin>=3 && pin<=11 && pin!=4 && pin!=7 && pin!=8)) error(PSTR("'analogwrite' invalid pin"));
#elif defined(__AVR_ATmega2560__)
  if (!((pin>=2 && pin<=13) || (pin>=44 && pin <=46))) error(PSTR("'analogwrite' invalid pin"));
#elif defined(__AVR_ATmega1284P__)
  if (!(pin==3 || pin==4 || pin==6 || pin==7 || (pin>=12 && pin<=15))) error(PSTR("'analogwrite' invalid pin"));
#endif
}

// Note

const uint8_t scale[] PROGMEM = {239,226,213,201,190,179,169,160,151,142,134,127};

void playnote (int pin, int note, int octave) {
  #if defined(__AVR_ATmega328P__)
  if (pin == 3) {
    DDRD = DDRD | 1<<DDD3; // PD3 (Arduino D3) as output
    TCCR2A = 0<<COM2A0 | 1<<COM2B0 | 2<<WGM20; // Toggle OC2B on match
  } else if (pin == 11) {
    DDRB = DDRB | 1<<DDB3; // PB3 (Arduino D11) as output
    TCCR2A = 1<<COM2A0 | 0<<COM2B0 | 2<<WGM20; // Toggle OC2A on match
  } else error(PSTR("'note' pin not supported"));
  int prescaler = 9 - octave - note/12;
  if (prescaler<3 || prescaler>6) error(PSTR("'note' octave out of range"));
  OCR2A = pgm_read_byte(&scale[note%12]) - 1;
  TCCR2B = 0<<WGM22 | prescaler<<CS20;

  #elif defined(__AVR_ATmega2560__)
  if (pin == 9) {
    DDRH = DDRH | 1<<DDH6; // PH6 (Arduino D9) as output
    TCCR2A = 0<<COM2A0 | 1<<COM2B0 | 2<<WGM20; // Toggle OC2B on match
  } else if (pin == 10) {
    DDRB = DDRB | 1<<DDB4; // PB4 (Arduino D10) as output
    TCCR2A = 1<<COM2A0 | 0<<COM2B0 | 2<<WGM20; // Toggle OC2A on match
  } else error(PSTR("'note' pin not supported"));
  int prescaler = 9 - octave - note/12;
  if (prescaler<3 || prescaler>6) error(PSTR("'note' octave out of range"));
  OCR2A = pgm_read_byte(&scale[note%12]) - 1;
  TCCR2B = 0<<WGM22 | prescaler<<CS20;

  #elif defined(__AVR_ATmega1284P__)
  if (pin == 14) {
    DDRD = DDRD | 1<<DDD6; // PD6 (Arduino D14) as output
    TCCR2A = 0<<COM2A0 | 1<<COM2B0 | 2<<WGM20; // Toggle OC2B on match
  } else if (pin == 15) {
    DDRD = DDRD | 1<<DDD7; // PD7 (Arduino D15) as output
    TCCR2A = 1<<COM2A0 | 0<<COM2B0 | 2<<WGM20; // Toggle OC2A on match
  } else error(PSTR("'note' pin not supported"));
  int prescaler = 9 - octave - note/12;
  if (prescaler<3 || prescaler>6) error(PSTR("'note' octave out of range"));
  OCR2A = pgm_read_byte(&scale[note%12]) - 1;
  TCCR2B = 0<<WGM22 | prescaler<<CS20;
  #endif
}

void nonote (int pin) {
  (void) pin;
  TCCR2B = 0<<WGM22 | 0<<CS20;
}

// Special forms

object *sp_quote (object *args, object *env) {
  (void) env;
  return first(args);
}

object *sp_defun (object *args, object *env) {
  (void) env;
  object *var = first(args);
  if (var->type != SYMBOL) error2(var, PSTR("is not a symbol"));
  object *val = cons(symbol(LAMBDA), cdr(args));
  object *pair = value(var->name,GlobalEnv);
  if (pair != NULL) { cdr(pair) = val; return var; }
  push(cons(var, val), GlobalEnv);
  return var;
}

object *sp_defvar (object *args, object *env) {
  object *var = first(args);
  if (var->type != SYMBOL) error2(var, PSTR("is not a symbol"));
  object *val = eval(second(args), env);
  object *pair = value(var->name,GlobalEnv);
  if (pair != NULL) { cdr(pair) = val; return var; }
  push(cons(var, val), GlobalEnv);
  return var;
}

object *sp_setq (object *args, object *env) {
  object *arg = eval(second(args), env);
  object *pair = findvalue(first(args), env);
  cdr(pair) = arg;
  return arg;
}

object *sp_loop (object *args, object *env) {
  clrflag(RETURNFLAG);
  object *start = args;
  for (;;) {
    args = start;
    while (args != NULL) {
      object *result = eval(car(args),env);
      if (tstflag(RETURNFLAG)) {
        clrflag(RETURNFLAG);
        return result;
      }
      args = cdr(args);
    }
  }
}

object *sp_push (object *args, object *env) {
  object *item = eval(first(args), env);
  object **loc = place(second(args), env);
  push(item, *loc);
  return *loc;
}

object *sp_pop (object *args, object *env) {
  object **loc = place(first(args), env);
  object *result = car(*loc);
  pop(*loc);
  return result;
}

object *sp_incf (object *args, object *env) {
  object **loc = place(first(args), env);
  int increment = 1;
  int result = integer(*loc);
  args = cdr(args);
  if (args != NULL) increment = integer(eval(first(args), env));
  #if defined(checkoverflow)
  if (increment < 1) { if (-32768 - increment > result) error(PSTR("'incf' arithmetic overflow")); }
  else { if (32767 - increment < result) error(PSTR("'incf' arithmetic overflow")); }
  #endif
  result = result + increment;
  *loc = number(result);
  return *loc;
}

object *sp_decf (object *args, object *env) {
  object **loc = place(first(args), env);
  int decrement = 1;
  int result = integer(*loc);
  args = cdr(args);
  if (args != NULL) decrement = integer(eval(first(args), env));
  #if defined(checkoverflow)
  if (decrement < 1) { if (32767 + decrement < result) error(PSTR("'decf' arithmetic overflow")); }
  else { if (-32768 + decrement > result) error(PSTR("'decf' arithmetic overflow")); }
  #endif
  result = result - decrement;
  *loc = number(result);
  return *loc;
}

object *sp_setf (object *args, object *env) {
  object **loc = place(first(args), env);
  object *result = eval(second(args), env);
  *loc = result;
  return result;
}

object *sp_dolist (object *args, object *env) {
  object *params = first(args);
  object *var = first(params);
  object *result;
  object *list = eval(second(params), env);
  if (!listp(list)) error(PSTR("'dolist' argument is not a list"));
  push(list, GCStack); // Don't GC the list
  object *pair = cons(var,nil);
  push(pair,env);
  params = cdr(cdr(params));
  object *forms = cdr(args);
  while (list != NULL) {
    cdr(pair) = first(list);
    list = cdr(list);
    result = eval(tf_progn(forms,env), env);
    if (tstflag(RETURNFLAG)) {
      clrflag(RETURNFLAG);
      return result;
    }
  }
  cdr(pair) = nil;
  pop(GCStack);
  if (params == NULL) return nil;
  return eval(car(params), env);
}

object *sp_dotimes (object *args, object *env) {
  object *params = first(args);
  object *var = first(params);
  object *result;
  int count = integer(eval(second(params), env));
  int index = 0;
  params = cdr(cdr(params));
  object *pair = cons(var,number(0));
  push(pair,env);
  object *forms = cdr(args);
  while (index < count) {
    cdr(pair) = number(index);
    index++;
    result = eval(tf_progn(forms,env), env);
    if (tstflag(RETURNFLAG)) {
      clrflag(RETURNFLAG);
      return result;
    }
  }
  cdr(pair) = number(index);
  if (params == NULL) return nil;
  return eval(car(params), env);
}

object *sp_trace (object *args, object *env) {
  (void) env;
  while (args != NULL) {
      trace(first(args)->name);
      args = cdr(args);
    }
  int i = 0;
  while (i < TRACEMAX) {
    if (TraceFn[i] != 0) args = cons(symbol(TraceFn[i]), args);
    i++;
  }
  return args;
}

object *sp_untrace (object *args, object *env) {
  (void) env;
  if (args == NULL) {
    int i = 0;
    while (i < TRACEMAX) {
      if (TraceFn[i] != 0) args = cons(symbol(TraceFn[i]), args);
      TraceFn[i] = 0;
      i++;
    }
  } else {
    while (args != NULL) {
      untrace(first(args)->name);
      args = cdr(args);
    }
  }
  return args;
}

object *sp_formillis (object *args, object *env) {
  object *param = first(args);
  unsigned long start = millis();
  unsigned long now, total = 0;
  if (param != NULL) total = integer(first(param));
  eval(tf_progn(cdr(args),env), env);
  do now = millis() - start; while (now < total);
  if (now <= 32767) return number(now);
  return nil;
}

object *sp_withserial (object *args, object *env) {
  object *params = first(args);
  object *var = first(params);
  int address = integer(eval(second(params), env));
  params = cddr(params);
  #if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega2560__)
  int baud = 96;
  if (params != NULL) baud = integer(eval(first(params), env));
  #endif
  object *pair = cons(var, stream(SERIALSTREAM, address));
  push(pair,env);
  #if defined(__AVR_ATmega1284P__)
  if (address == 1) Serial1.begin(baud*100);
  #elif defined(__AVR_ATmega2560__)
  if (address == 1) Serial1.begin(baud*100);
  else if (address == 2) Serial2.begin(baud*100);
  else if (address == 3) Serial3.begin(baud*100);
  #endif
  object *forms = cdr(args);
  object *result = eval(tf_progn(forms,env), env);
  #if defined(__AVR_ATmega1284P__)
  if (address == 1) Serial1.end();
  #elif defined(__AVR_ATmega2560__)
  if (address == 1) Serial1.end();
  else if (address == 2) Serial2.end();
  else if (address == 3) Serial3.end();
  #endif
  return result;
}

object *sp_withi2c (object *args, object *env) {
  object *params = first(args);
  object *var = first(params);
  int address = integer(eval(second(params), env));
  params = cddr(params);
  int read = 0; // Write
  I2CCount = 0;
  if (params != NULL) {
    object *rw = eval(first(params), env);
    if (numberp(rw)) I2CCount = integer(rw);
    read = (rw != NULL);
  }
  I2Cinit(1); // Pullups
  object *pair = cons(var, (I2Cstart(address, read)) ? stream(I2CSTREAM, address) : nil);
  push(pair,env);
  object *forms = cdr(args);
  object *result = eval(tf_progn(forms,env), env);
  I2Cstop(read);
  return result;
}

object *sp_withspi (object *args, object *env) {
  object *params = first(args);
  object *var = first(params);
  int pin = integer(eval(second(params), env));
  int divider = 0, mode = 0, bitorder = 1;
  object *pair = cons(var, stream(SPISTREAM, pin));
  push(pair,env);
  SPI.begin();
  params = cddr(params);
  if (params != NULL) {
    int d = integer(eval(first(params), env));
    if (d<1 || d>7) error(PSTR("'with-spi' invalid divider"));
    if (d == 7) divider = 3;
    else if (d & 1) divider = (d>>1) + 4;
    else divider = (d>>1) - 1;
    params = cdr(params);
    if (params != NULL) {
      bitorder = (eval(first(params), env) == NULL);
      params = cdr(params);
      if (params != NULL) mode = integer(eval(first(params), env));
    }
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  SPI.setBitOrder(bitorder);
  SPI.setClockDivider(divider);
  SPI.setDataMode(mode);
  object *forms = cdr(args);
  object *result = eval(tf_progn(forms,env), env);
  digitalWrite(pin, HIGH);
  SPI.end();
  return result;
}

// Tail-recursive forms

object *tf_progn (object *args, object *env) {
  if (args == NULL) return nil;
  object *more = cdr(args);
  while (more != NULL) {
    object *result = eval(car(args),env);
    if (tstflag(RETURNFLAG)) return result;
    args = more;
    more = cdr(args);
  }
  return car(args);
}

object *tf_return (object *args, object *env) {
  setflag(RETURNFLAG);
  return tf_progn(args, env);
}

object *tf_if (object *args, object *env) {
  if (eval(first(args), env) != nil) return second(args);
  args = cddr(args);
  return (args != NULL) ? first(args) : nil;
}

object *tf_cond (object *args, object *env) {
  while (args != NULL) {
    object *clause = first(args);
    object *test = eval(first(clause), env);
    object *forms = cdr(clause);
    if (test != nil) {
      if (forms == NULL) return test; else return tf_progn(forms, env);
    }
    args = cdr(args);
  }
  return nil;
}

object *tf_when (object *args, object *env) {
  if (eval(first(args), env) != nil) return tf_progn(cdr(args),env);
  else return nil;
}

object *tf_unless (object *args, object *env) {
  if (eval(first(args), env) != nil) return nil;
  else return tf_progn(cdr(args),env);
}

object *tf_and (object *args, object *env) {
  if (args == NULL) return tee;
  object *more = cdr(args);
  while (more != NULL) {
    if (eval(car(args), env) == NULL) return nil;
    args = more;
    more = cdr(args);
  }
  return car(args);
}

object *tf_or (object *args, object *env) {
  object *more = cdr(args);
  while (more != NULL) {
    object *result = eval(car(args), env);
    if (result != NULL) return result;
    args = more;
    more = cdr(args);
  }
  return car(args);
}

// Core functions

object *fn_not (object *args, object *env) {
  (void) env;
  return (first(args) == nil) ? tee : nil;
}

object *fn_cons (object *args, object *env) {
  (void) env;
  return cons(first(args),second(args));
}

object *fn_atom (object *args, object *env) {
  (void) env;
  return atom(first(args)) ? tee : nil;
}

object *fn_listp (object *args, object *env) {
  (void) env;
  return listp(first(args)) ? tee : nil;
}

object *fn_consp (object *args, object *env) {
  (void) env;
  return consp(first(args)) ? tee : nil;
}

object *fn_numberp (object *args, object *env) {
  (void) env;
  return numberp(first(args)) ? tee : nil;
}

object *fn_symbolp (object *args, object *env) {
  (void) env;
  return symbolp(first(args)) ? tee : nil;
}

object *fn_streamp (object *args, object *env) {
  (void) env;
  return streamp(first(args)) ? tee : nil;
}

object *fn_eq (object *args, object *env) {
  (void) env;
  return eq(first(args), second(args)) ? tee : nil;
}

// List functions

object *fn_car (object *args, object *env) {
  (void) env;
  return carx(first(args));
}

object *fn_cdr (object *args, object *env) {
  (void) env;
  return cdrx(first(args));
}

object *fn_caar (object *args, object *env) {
  (void) env;
  return carx(carx(first(args)));
}

object *fn_cadr (object *args, object *env) {
  (void) env;
  return carx(cdrx(first(args)));
}

object *fn_cdar (object *args, object *env) {
  (void) env;
  return cdrx(carx(first(args)));
}

object *fn_cddr (object *args, object *env) {
  (void) env;
  return cdrx(cdrx(first(args)));
}

object *fn_caaar (object *args, object *env) {
  (void) env;
  return carx(carx(carx(first(args))));
}

object *fn_caadr (object *args, object *env) {
  (void) env;
  return carx(carx(cdrx(first(args))));
}

object *fn_cadar (object *args, object *env) {
  (void) env;
  return carx(cdrx(carx(first(args))));
}

object *fn_caddr (object *args, object *env) {
  (void) env;
  return carx(cdrx(cdrx(first(args))));
}

object *fn_cdaar (object *args, object *env) {
  (void) env;
  return cdrx(carx(carx(first(args))));
}

object *fn_cdadr (object *args, object *env) {
  (void) env;
  return cdrx(carx(cdrx(first(args))));
}

object *fn_cddar (object *args, object *env) {
  (void) env;
  return cdrx(cdrx(carx(first(args))));
}

object *fn_cdddr (object *args, object *env) {
  (void) env;
  return cdrx(cdrx(cdrx(first(args))));
}

object *fn_length (object *args, object *env) {
  (void) env;
  object *arg = first(args);
  if (listp(arg)) return number(listlength(arg));
  if (!stringp(arg)) error(PSTR("'length' argument is not a list or string"));
  return number(stringlength(arg));
}

object *fn_list (object *args, object *env) {
  (void) env;
  return args;
}

object *fn_reverse (object *args, object *env) {
  (void) env;
  object *list = first(args);
  if (!listp(list)) error(PSTR("'reverse' argument is not a list"));
  object *result = NULL;
  while (list != NULL) {
    push(first(list),result);
    list = cdr(list);
  }
  return result;
}

object *fn_nth (object *args, object *env) {
  (void) env;
  int n = integer(first(args));
  object *list = second(args);
  if (!listp(list)) error(PSTR("'nth' second argument is not a list"));
  while (list != NULL) {
    if (n == 0) return car(list);
    list = cdr(list);
    n--;
  }
  return nil;
}

object *fn_assoc (object *args, object *env) {
  (void) env;
  object *key = first(args);
  object *list = second(args);
  if (!listp(list)) error(PSTR("'assoc' second argument is not a list"));
  return assoc(key,list);
}

object *fn_member (object *args, object *env) {
  (void) env;
  object *item = first(args);
  object *list = second(args);
  if (!listp(list)) error(PSTR("'member' second argument is not a list"));
  while (list != NULL) {
    if (eq(item,car(list))) return list;
    list = cdr(list);
  }
  return nil;
}

object *fn_apply (object *args, object *env) {
  object *previous = NULL;
  object *last = args;
  while (cdr(last) != NULL) {
    previous = last;
    last = cdr(last);
  }
  if (!listp(car(last))) error(PSTR("'apply' last argument is not a list"));
  cdr(previous) = car(last);
  return apply(first(args), cdr(args), &env);
}

object *fn_funcall (object *args, object *env) {
  return apply(first(args), cdr(args), &env);
}

object *fn_append (object *args, object *env) {
  (void) env;
  object *head = NULL;
  object *tail = NULL;
  while (args != NULL) {
    object *list = first(args);
    if (!listp(list)) error(PSTR("'append' argument is not a list"));
    while (list != NULL) {
      object *obj = cons(first(list),NULL);
      if (head == NULL) {
        head = obj;
        tail = obj;
      } else {
        cdr(tail) = obj;
        tail = obj;
      }
      list = cdr(list);
    }
    args = cdr(args);
  }
  return head;
}

object *fn_mapc (object *args, object *env) {
  object *function = first(args);
  object *list1 = second(args);
  object *result = list1;
  if (!listp(list1)) error(PSTR("'mapc' second argument is not a list"));
  object *list2 = cddr(args);
  if (list2 != NULL) {
    list2 = car(list2);
    if (!listp(list2)) error(PSTR("'mapc' third argument is not a list"));
  }
  if (list2 != NULL) {
    while (list1 != NULL && list2 != NULL) {
      apply(function, cons(car(list1),cons(car(list2),NULL)), &env);
      list1 = cdr(list1);
      list2 = cdr(list2);
    }
  } else {
    while (list1 != NULL) {
      apply(function, cons(car(list1),NULL), &env);
      list1 = cdr(list1);
    }
  }
  return result;
}

object *fn_mapcar (object *args, object *env) {
  object *function = first(args);
  object *list1 = second(args);
  if (!listp(list1)) error(PSTR("'mapcar' second argument is not a list"));
  object *list2 = cddr(args);
  if (list2 != NULL) {
    list2 = car(list2);
    if (!listp(list2)) error(PSTR("'mapcar' third argument is not a list"));
  }
  object *head = NULL;
  object *tail = NULL;
  if (list2 != NULL) {
    while (list1 != NULL && list2 != NULL) {
      object *result = apply(function, cons(car(list1),cons(car(list2),NULL)), &env);
      object *obj = cons(result,NULL);
      if (head == NULL) {
        head = obj;
        push(head,GCStack);
        tail = obj;
      } else {
        cdr(tail) = obj;
        tail = obj;
      }
      list1 = cdr(list1);
      list2 = cdr(list2);
    }
  } else {
    while (list1 != NULL) {
      object *result = apply(function, cons(car(list1),NULL), &env);
      object *obj = cons(result,NULL);
      if (head == NULL) {
        head = obj;
        push(head,GCStack);
        tail = obj;
      } else {
        cdr(tail) = obj;
        tail = obj;
      }
      list1 = cdr(list1);
    }
  }
  pop(GCStack);
  return head;
}

// Arithmetic functions

object *fn_add (object *args, object *env) {
  (void) env;
  int result = 0;
  while (args != NULL) {
    int temp = integer(car(args));
    #if defined(checkoverflow)
    if (temp < 1) { if (-32768 - temp > result) error(PSTR("'+' arithmetic overflow")); }
    else { if (32767 - temp < result) error(PSTR("'+' arithmetic overflow")); }
    #endif
    result = result + temp;
    args = cdr(args);
  }
  return number(result);
}

object *fn_subtract (object *args, object *env) {
  (void) env;
  int result = integer(car(args));
  args = cdr(args);
  if (args == NULL) {
    #if defined(checkoverflow)
    if (result == -32768) error(PSTR("'-' arithmetic overflow"));
    #endif
    return number(-result);
  }
  while (args != NULL) {
    int temp = integer(car(args));
    #if defined(checkoverflow)
    if (temp < 1) { if (32767 + temp < result) error(PSTR("'-' arithmetic overflow")); }
    else { if (-32768 + temp > result) error(PSTR("'-' arithmetic overflow")); }
    #endif
    result = result - temp;
    args = cdr(args);
  }
  return number(result);
}

object *fn_multiply (object *args, object *env) {
  (void) env;
  int result = 1;
  while (args != NULL){
    #if defined(checkoverflow)
    signed long temp = (signed long) result * integer(car(args));
    if ((temp > 32767) || (temp < -32768)) error(PSTR("'*' arithmetic overflow"));
    result = temp;
    #else
    result = result * integer(car(args));
    #endif
    args = cdr(args);
  }
  return number(result);
}

object *fn_divide (object *args, object *env) {
  (void) env;
  int result = integer(first(args));
  args = cdr(args);
  while (args != NULL) {
    int arg = integer(car(args));
    if (arg == 0) error(PSTR("Division by zero"));
    #if defined(checkoverflow)
    if ((result == -32768) && (arg == -1)) error(PSTR("'/' arithmetic overflow"));
    #endif
    result = result / arg;
    args = cdr(args);
  }
  return number(result);
}

object *fn_mod (object *args, object *env) {
  (void) env;
  int arg1 = integer(first(args));
  int arg2 = integer(second(args));
  if (arg2 == 0) error(PSTR("Division by zero"));
  int r = arg1 % arg2;
  if ((arg1<0) != (arg2<0)) r = r + arg2;
  return number(r);
}

object *fn_oneplus (object *args, object *env) {
  (void) env;
  int result = integer(first(args));
  #if defined(checkoverflow)
  if (result == 32767) error(PSTR("'1+' arithmetic overflow"));
  #endif
  return number(result + 1);
}

object *fn_oneminus (object *args, object *env) {
  (void) env;
  int result = integer(first(args));
  #if defined(checkoverflow)
  if (result == -32768) error(PSTR("'1-' arithmetic overflow"));
  #endif
  return number(result - 1);
}

object *fn_abs (object *args, object *env) {
  (void) env;
  int result = integer(first(args));
  #if defined(checkoverflow)
  if (result == -32768) error(PSTR("'abs' arithmetic overflow"));
  #endif
  return number(abs(result));
}

object *fn_random (object *args, object *env) {
  (void) env;
  int arg = integer(first(args));
  return number(random(arg));
}

object *fn_max (object *args, object *env) {
  (void) env;
  int result = integer(first(args));
  args = cdr(args);
  while (args != NULL) {
    result = max(result,integer(car(args)));
    args = cdr(args);
  }
  return number(result);
}

object *fn_min (object *args, object *env) {
  (void) env;
  int result = integer(first(args));
  args = cdr(args);
  while (args != NULL) {
    result = min(result,integer(car(args)));
    args = cdr(args);
  }
  return number(result);
}

// Arithmetic comparisons

object *fn_numeq (object *args, object *env) {
  (void) env;
  int arg1 = integer(first(args));
  args = cdr(args);
  while (args != NULL) {
    int arg2 = integer(first(args));
    if (!(arg1 == arg2)) return nil;
    arg1 = arg2;
    args = cdr(args);
  }
  return tee;
}

object *fn_less (object *args, object *env) {
  (void) env;
  int arg1 = integer(first(args));
  args = cdr(args);
  while (args != NULL) {
    int arg2 = integer(first(args));
    if (!(arg1 < arg2)) return nil;
    arg1 = arg2;
    args = cdr(args);
  }
  return tee;
}

object *fn_lesseq (object *args, object *env) {
  (void) env;
  int arg1 = integer(first(args));
  args = cdr(args);
  while (args != NULL) {
    int arg2 = integer(first(args));
    if (!(arg1 <= arg2)) return nil;
    arg1 = arg2;
    args = cdr(args);
  }
  return tee;
}

object *fn_greater (object *args, object *env) {
  (void) env;
  int arg1 = integer(first(args));
  args = cdr(args);
  while (args != NULL) {
    int arg2 = integer(first(args));
    if (!(arg1 > arg2)) return nil;
    arg1 = arg2;
    args = cdr(args);
  }
  return tee;
}

object *fn_greatereq (object *args, object *env) {
  (void) env;
  int arg1 = integer(first(args));
  args = cdr(args);
  while (args != NULL) {
    int arg2 = integer(first(args));
    if (!(arg1 >= arg2)) return nil;
    arg1 = arg2;
    args = cdr(args);
  }
  return tee;
}

object *fn_noteq (object *args, object *env) {
  (void) env;
  while (args != NULL) {   
    object *nargs = args;
    int arg1 = integer(first(nargs));
    nargs = cdr(nargs);
    while (nargs != NULL) {
       int arg2 = integer(first(nargs));
       if (arg1 == arg2) return nil;
       nargs = cdr(nargs);
    }
    args = cdr(args);
  }
  return tee;
}

object *fn_plusp (object *args, object *env) {
  (void) env;
  int arg = integer(first(args));
  if (arg > 0) return tee;
  else return nil;
}

object *fn_minusp (object *args, object *env) {
  (void) env;
  int arg = integer(first(args));
  if (arg < 0) return tee;
  else return nil;
}

object *fn_zerop (object *args, object *env) {
  (void) env;
  int arg = integer(first(args));
  if (arg == 0) return tee;
  else return nil;
}

object *fn_oddp (object *args, object *env) {
  (void) env;
  int arg = integer(first(args));
  if ((arg & 1) == 1) return tee;
  else return nil;
}

object *fn_evenp (object *args, object *env) {
  (void) env;
  int arg = integer(first(args));
  if ((arg & 1) == 0) return tee;
  else return nil;
}

// Characters

object *fn_char (object *args, object *env) {
  (void) env;
  char c = nthchar(first(args), integer(second(args)));
  if (c == 0) error(PSTR("'char' index out of range"));
  return character(c);
}

object *fn_charcode (object *args, object *env) {
  (void) env;
  return number(fromchar(first(args)));
}

object *fn_codechar (object *args, object *env) {
  (void) env;
  return character(integer(first(args)));
}

object *fn_characterp (object *args, object *env) {
  (void) env;
  return characterp(first(args)) ? tee : nil;
}

// Strings

object *fn_stringp (object *args, object *env) {
  (void) env;
  return stringp(first(args)) ? tee : nil;
}

bool stringcompare (object *args, bool lt, bool gt, bool eq) {
  object *arg1 = first(args);
  object *arg2 = second(args);
  if (!stringp(arg1) || !stringp(arg2)) error(PSTR("String compare argument is not a string"));
  arg1 = cdr(arg1);
  arg2 = cdr(arg2);
  while ((arg1 != NULL) || (arg2 != NULL)) {
    if (arg1 == NULL) return lt;
    if (arg2 == NULL) return gt;
    if (arg1->integer < arg2->integer) return lt;
    if (arg1->integer > arg2->integer) return gt;
    arg1 = car(arg1);
    arg2 = car(arg2);
  }
  return eq;
}

object *fn_stringeq (object *args, object *env) {
  (void) env;
  return stringcompare(args, false, false, true) ? tee : nil;
}

object *fn_stringless (object *args, object *env) {
  (void) env;
  return stringcompare(args, true, false, false) ? tee : nil;
}

object *fn_stringgreater (object *args, object *env) {
  (void) env;
  return stringcompare(args, false, true, false) ? tee : nil;
}

object *fn_sort (object *args, object *env) {
  if (first(args) == NULL) return nil;
  object *list = cons(nil,first(args));
  push(list,GCStack);
  object *predicate = second(args);
  object *compare = cons(NULL,cons(NULL,NULL));
  object *ptr = cdr(list);
  while (cdr(ptr) != NULL) {
    object *go = list;
    while (go != ptr) {
      car(compare) = car(cdr(ptr));
      car(cdr(compare)) = car(cdr(go));
      if (apply(predicate, compare, &env)) break;
      go = cdr(go);
    }
    if (go != ptr) {
      object *obj = cdr(ptr);
      cdr(ptr) = cdr(obj);
      cdr(obj) = cdr(go);
      cdr(go) = obj;
    } else ptr = cdr(ptr);
  }
  pop(GCStack);
  return cdr(list);
}

object *fn_stringfn (object *args, object *env) {
  (void) env;
  object *arg = first(args);
  int type = arg->type;
  if (type == STRING) return arg;
  object *obj = myalloc();
  obj->type = STRING;
  if (type == CHARACTER) {
    object *cell = myalloc();
    cell->car = NULL;
    cell->integer = fromchar(arg)<<8;
    obj->cdr = cell;
  } else if (type == SYMBOL) {
    char *s = name(arg);
    char ch = *s++;
    object *head = NULL;
    int chars = 0;
    while (ch) {
      if (ch == '\\') ch = *s++;
      buildstring(ch, &chars, &head);
      ch = *s++;
    }
    obj->cdr = head;
  } else error(PSTR("Cannot convert to string"));
  return obj;
}

object *fn_concatenate (object *args, object *env) {
  (void) env;
  object *arg = first(args);
  symbol_t name = arg->name;
  if (name != STRINGFN) error(PSTR("Only string result supported"));
  args = cdr(args);
  object *result = myalloc();
  result->type = STRING;
  object *head = NULL;
  int chars = 0;
  while (args != NULL) {
    object *obj = first(args);
    if (obj->type != STRING) error2(obj, PSTR("not a string"));
    obj = cdr(obj);
    while (obj != NULL) {
      int pair = obj->integer;
       while (pair != 0) {
         char ch = pair>>8 & 0xFF;
         buildstring(ch, &chars, &head);
         pair = pair<<8;
      }
      obj = car(obj);
    }
    args = cdr(args);
  }
  result->cdr = head;
  return result;
}

object *fn_subseq (object *args, object *env) {
  (void) env;
  object *arg = first(args);
  if (!stringp(arg)) error(PSTR("'subseq' first argument is not a string"));
  int start = integer(second(args));
  int end;
  args = cddr(args);
  if (args != NULL) end = integer(car(args)); else end = stringlength(arg);
  object *result = myalloc();
  result->type = STRING;
  object *head = NULL;
  int chars = 0;
  for (int i=start; i<end; i++) {
    char ch = nthchar(arg, i);
    if (ch == 0) error(PSTR("'subseq' index out of range"));
    buildstring(ch, &chars, &head);
  }
  result->cdr = head;
  return result;
}

char gstr () {
  if (LastChar) { 
    char temp = LastChar;
    LastChar = 0;
    return temp;
  }
  char c = nthchar(GlobalString, GlobalStringIndex++);
  return (c != 0) ? c : '\n';
}

object *fn_readfromstring (object *args, object *env) {   
  (void) env;
  object *arg = first(args);
  if (!stringp(arg)) error(PSTR("'read-from-string' argument is not a string"));
  GlobalString = arg;
  GlobalStringIndex = 0;
  return read(gstr);
}

void pstr (char c) {
  buildstring(c, &GlobalStringIndex, &GlobalString);
}
 
object *fn_princtostring (object *args, object *env) {   
  (void) env;
  object *arg = first(args);
  object *obj = myalloc();
  obj->type = STRING;
  GlobalString = NULL;
  GlobalStringIndex = 0;
  char temp = PrintReadably;
  PrintReadably = 0;
  printobject(arg, pstr);
  PrintReadably = temp;
  obj->cdr = GlobalString;
  return obj;
}

object *fn_prin1tostring (object *args, object *env) {   
  (void) env;
  object *arg = first(args);
  object *obj = myalloc();
  obj->type = STRING;
  GlobalString = NULL;
  GlobalStringIndex = 0;
  printobject(arg, pstr);
  obj->cdr = GlobalString;
  return obj;
}

// Bitwise operators

object *fn_logand (object *args, object *env) {
  (void) env;
  unsigned int result = 0xFFFF;
  while (args != NULL) {
    result = result & integer(first(args));
    args = cdr(args);
  }
  return number(result);
}

object *fn_logior (object *args, object *env) {
  (void) env;
  unsigned int result = 0;
  while (args != NULL) {
    result = result | integer(first(args));
    args = cdr(args);
  }
  return number(result);
}

object *fn_logxor (object *args, object *env) {
  (void) env;
  unsigned int result = 0;
  while (args != NULL) {
    result = result ^ integer(first(args));
    args = cdr(args);
  }
  return number(result);
}

object *fn_lognot (object *args, object *env) {
  (void) env;
  int result = integer(car(args));
  return number(~result);
}

object *fn_ash (object *args, object *env) {
  (void) env;
  int value = integer(first(args));
  int count = integer(second(args));
  if (count >= 0)
    return number(value << count);
  else
    return number(value >> abs(count));
}

object *fn_logbitp (object *args, object *env) {
  (void) env;
  int index = integer(first(args));
  int value = integer(second(args));
  return (bitRead(value, index) == 1) ? tee : nil;
}

// System functions

object *fn_eval (object *args, object *env) {
  return eval(first(args), env);
}

object *fn_globals (object *args, object *env) {
  (void) args;
  if (GlobalEnv == NULL) return nil;
  return fn_mapcar(cons(symbol(CAR),cons(GlobalEnv,nil)), env);
}

object *fn_locals (object *args, object *env) {
  (void) args;
  return env;
}

object *fn_makunbound (object *args, object *env) {
  (void) env;
  object *key = first(args);
  deletesymbol(key->name);
  return delassoc(key, &GlobalEnv);
}

object *fn_break (object *args, object *env) {
  (void) args;
  pfstring(PSTR("\rBreak!\r"), pserial);
  BreakLevel++;
  repl(env);
  BreakLevel--;
  return nil;
}

object *fn_read (object *args, object *env) {
  (void) args;
  (void) env;
  return read(gserial);
}

object *fn_prin1 (object *args, object *env) {
  (void) env;
  object *obj = first(args);
  printobject(obj, pserial);
  return obj;
}

object *fn_print (object *args, object *env) {
  (void) env;
  pln(pserial);
  object *obj = first(args);
  printobject(obj, pserial);
  pserial(' ');
  return obj;
}

object *fn_princ (object *args, object *env) {
  (void) env;
  object *obj = first(args);
  char temp = PrintReadably;
  PrintReadably = 0;
  printobject(obj, pserial);
  PrintReadably = temp;
  return obj;
}

object *fn_terpri (object *args, object *env) {
  (void) args, (void) env;
  pln(pserial);
  return nil;
}

object *fn_readbyte (object *args, object *env) {
  (void) env;
  return number((gstreamfun(args))());
}

object *fn_readline (object *args, object *env) {
  (void) env;
  return readstring('\n', gstreamfun(args));
}

object *fn_writebyte (object *args, object *env) {
  (void) env;
  int value = integer(first(args));
  (pstreamfun(cdr(args)))(value);
  return nil;
}

object *fn_writestring (object *args, object *env) {
  (void) env;
  object *obj = first(args);
  char temp = PrintReadably;
  PrintReadably = 0;
  printstring(obj, pstreamfun(cdr(args)));
  PrintReadably = temp;
  return nil;
}

object *fn_writeline (object *args, object *env) {
  (void) env;
  object *obj = first(args);
  char temp = PrintReadably;
  PrintReadably = 0;
  printstring(obj, pstreamfun(cdr(args)));
  (pstreamfun(cdr(args)))('\n');
  PrintReadably = temp;
  return nil;
}

object *fn_restarti2c (object *args, object *env) {
  (void) env;
  int stream = first(args)->integer;
  args = cdr(args);
  int read = 0; // Write
  I2CCount = 0;
  if (args != NULL) {
    object *rw = first(args);
    if (numberp(rw)) I2CCount = integer(rw);
    read = (rw != NULL);
  }
  int address = stream & 0xFF;
  if (stream>>8 != I2CSTREAM) error(PSTR("'restart' not i2c"));
  return I2Crestart(address, read) ? tee : nil;
}

object *fn_gc (object *obj, object *env) {
  unsigned long start = micros();
  int initial = Freespace;
  gc(obj, env);
  pfstring(PSTR("Space: "), pserial);
  pint(Freespace - initial, pserial);
  pfstring(PSTR(" bytes, Time: "), pserial);
  pint(micros() - start, pserial);
  pfstring(PSTR(" uS\r"), pserial);
  return nil;
}

object *fn_room (object *args, object *env) {
  (void) args, (void) env;
  return number(Freespace);
}

object *fn_saveimage (object *args, object *env) {
  if (args != NULL) args = eval(first(args), env);
  return number(saveimage(args));
}

object *fn_loadimage (object *args, object *env) {
  (void) args, (void) env;
  return number(loadimage());
}

object *fn_cls(object *args, object *env) {
  (void) args, (void) env;
  pserial(12);
  return nil;
}

// Arduino procedures

object *fn_pinmode (object *args, object *env) {
  (void) env;
  int pin = integer(first(args));
  object *mode = second(args);
  if (numberp(mode)) pinMode(pin, mode->integer);
  else pinMode(pin, (mode != nil));
  return nil;
}

object *fn_digitalread (object *args, object *env) {
  (void) env;
  int pin = integer(first(args));
  if(digitalRead(pin) != 0) return tee; else return nil;
}

object *fn_digitalwrite (object *args, object *env) {
  (void) env;
  int pin = integer(first(args));
  object *mode = second(args);
  digitalWrite(pin, (mode != nil));
  return mode;
}

object *fn_analogread (object *args, object *env) {
  (void) env;
  int pin = integer(first(args));
  checkanalogread(pin);
  return number(analogRead(pin));
}
 
object *fn_analogwrite (object *args, object *env) {
  (void) env;
  int pin = integer(first(args));
  checkanalogwrite(pin);
  object *value = second(args);
  analogWrite(pin, integer(value));
  return value;
}

object *fn_delay (object *args, object *env) {
  (void) env;
  object *arg1 = first(args);
  delay(integer(arg1));
  return arg1;
}

object *fn_millis (object *args, object *env) {
  (void) args, (void) env;
  return number(millis());
}

object *fn_note (object *args, object *env) {
  (void) env;
  static int pin = 255;
  if (args != NULL) {
    pin = integer(first(args));
    int note = integer(second(args));
    int octave = 0;
    if (cddr(args) != NULL) octave = integer(third(args));
    playnote(pin, note, octave);
  } else nonote(pin);
  return nil;
}

// Tree Editor

object *fn_edit (object *args, object *env) {
  object *fun = first(args);
  object *pair = findvalue(fun, env);
  clrflag(EXITEDITOR);
  object *arg = edit(eval(fun, env));
  cdr(pair) = arg;
  return arg;
}

object *edit (object *fun) {
  while (1) {
    if (tstflag(EXITEDITOR)) return fun;
    char c = gserial();
    if (c == 'q') setflag(EXITEDITOR);
    else if (c == 'b') return fun;
    else if (c == 'r') fun = read(gserial);
    else if (c == '\n') { pfl(pserial); superprint(fun, 0); pln(pserial); }
    else if (c == 'c') fun = cons(read(gserial), fun);
    else if (atom(fun)) pserial('!');
    else if (c == 'd') fun = cons(car(fun), edit(cdr(fun)));
    else if (c == 'a') fun = cons(edit(car(fun)), cdr(fun));
    else if (c == 'x') fun = cdr(fun);
    else pserial('?');
  }
}

// Pretty printer

const int PPINDENT = 2;
const int PPWIDTH = 80;

int atomwidth (object *obj) {
  if (obj == NULL) return 3;
  if (numberp(obj)) {
    int w = 1;
    int n = obj->integer;
    if (n < 0) { n = -n; w++; }
    while (n >= 10) { n = n/10; w++; }
    return w;
  }
  if (stringp(obj)) return stringlength(obj);
  if (characterp(obj)) return 3; // Interim solution
  int w = 0;
  char *s = name(obj);
  while (*s++) w++;
  return w;
}

boolean quoted (object *obj) {
  return (consp(obj) && (car(obj)->name == QUOTE) && consp(cdr(obj)) && (cddr(obj) == NULL));
}

int subwidth (object *obj, int w) {
  if (atom(obj)) return w - atomwidth(obj);
  if (quoted(obj)) return subwidthlist(car(cdr(obj)), w - 1);
  return subwidthlist(obj, w - 1);
}

int subwidthlist (object *form, int w) {
  while (form != NULL && w >= 0) {
    if (atom(form)) return w - (2 + atomwidth(form));
    w = subwidth(car(form), w - 1);
    form = cdr(form);
  }
  return w;
}

void superprint (object *form, int lm) {
  if (atom(form)) printobject(form, pserial);
  else if (quoted(form)) { pserial('\''); superprint(car(cdr(form)), lm + 1); }
  else if (subwidth(form, PPWIDTH - lm) >= 0) supersub(form, lm + PPINDENT, 0);
  else supersub(form, lm + PPINDENT, 1);
}

const int ppspecials = 8;
const char ppspecial[ppspecials] PROGMEM = { IF, SETQ, TEE, LET, LETSTAR, LAMBDA, WHEN, UNLESS };

void supersub (object *form, int lm, int super) {
  int special = 0, separate = 1;
  object *arg = car(form);
  if (symbolp(arg)) {
    int name = arg->name;
    if (name == DEFUN) special = 2;
    else for (int i=0; i<ppspecials; i++) {
      if (name == pgm_read_byte(&ppspecial[i])) { special = 1; break; }    
    } 
  }
  while (form != NULL) {
    if (atom(form)) { pfstring(PSTR(" . "), pserial); printobject(form, pserial); pserial(')'); return; }
    else if (separate) { pserial('('); separate = 0; }
    else if (special) { pserial(' '); special--; }
    else if (!super) pserial(' ');
    else { pln(pserial); indent(lm); }
    superprint(car(form), lm);
    form = cdr(form);   
  }
  pserial(')'); return;
}

object *fn_pprint (object *args, object *env) {
  (void) env;
  pln(pserial);
  superprint(first(args), 0);
  return symbol(NOTHING);
}

// Insert your own function definitions here

// Built-in procedure names - stored in PROGMEM

const char string0[] PROGMEM = "symbols";
const char string1[] PROGMEM = "nil";
const char string2[] PROGMEM = "t";
const char string3[] PROGMEM = "nothing";
const char string4[] PROGMEM = "&rest";
const char string5[] PROGMEM = "lambda";
const char string6[] PROGMEM = "let";
const char string7[] PROGMEM = "let*";
const char string8[] PROGMEM = "closure";
const char string9[] PROGMEM = "special_forms";
const char string10[] PROGMEM = "quote";
const char string11[] PROGMEM = "defun";
const char string12[] PROGMEM = "defvar";
const char string13[] PROGMEM = "setq";
const char string14[] PROGMEM = "loop";
const char string15[] PROGMEM = "push";
const char string16[] PROGMEM = "pop";
const char string17[] PROGMEM = "incf";
const char string18[] PROGMEM = "decf";
const char string19[] PROGMEM = "setf";
const char string20[] PROGMEM = "dolist";
const char string21[] PROGMEM = "dotimes";
const char string22[] PROGMEM = "trace";
const char string23[] PROGMEM = "untrace";
const char string24[] PROGMEM = "for-millis";
const char string25[] PROGMEM = "with-serial";
const char string26[] PROGMEM = "with-i2c";
const char string27[] PROGMEM = "with-spi";
const char string28[] PROGMEM = "tail_forms";
const char string29[] PROGMEM = "progn";
const char string30[] PROGMEM = "return";
const char string31[] PROGMEM = "if";
const char string32[] PROGMEM = "cond";
const char string33[] PROGMEM = "when";
const char string34[] PROGMEM = "unless";
const char string35[] PROGMEM = "and";
const char string36[] PROGMEM = "or";
const char string37[] PROGMEM = "functions";
const char string38[] PROGMEM = "not";
const char string39[] PROGMEM = "null";
const char string40[] PROGMEM = "cons";
const char string41[] PROGMEM = "atom";
const char string42[] PROGMEM = "listp";
const char string43[] PROGMEM = "consp";
const char string44[] PROGMEM = "numberp";
const char string45[] PROGMEM = "symbolp";
const char string46[] PROGMEM = "streamp";
const char string47[] PROGMEM = "eq";
const char string48[] PROGMEM = "car";
const char string49[] PROGMEM = "first";
const char string50[] PROGMEM = "cdr";
const char string51[] PROGMEM = "rest";
const char string52[] PROGMEM = "caar";
const char string53[] PROGMEM = "cadr";
const char string54[] PROGMEM = "second";
const char string55[] PROGMEM = "cdar";
const char string56[] PROGMEM = "cddr";
const char string57[] PROGMEM = "caaar";
const char string58[] PROGMEM = "caadr";
const char string59[] PROGMEM = "cadar";
const char string60[] PROGMEM = "caddr";
const char string61[] PROGMEM = "third";
const char string62[] PROGMEM = "cdaar";
const char string63[] PROGMEM = "cdadr";
const char string64[] PROGMEM = "cddar";
const char string65[] PROGMEM = "cdddr";
const char string66[] PROGMEM = "length";
const char string67[] PROGMEM = "list";
const char string68[] PROGMEM = "reverse";
const char string69[] PROGMEM = "nth";
const char string70[] PROGMEM = "assoc";
const char string71[] PROGMEM = "member";
const char string72[] PROGMEM = "apply";
const char string73[] PROGMEM = "funcall";
const char string74[] PROGMEM = "append";
const char string75[] PROGMEM = "mapc";
const char string76[] PROGMEM = "mapcar";
const char string77[] PROGMEM = "+";
const char string78[] PROGMEM = "-";
const char string79[] PROGMEM = "*";
const char string80[] PROGMEM = "/";
const char string81[] PROGMEM = "mod";
const char string82[] PROGMEM = "1+";
const char string83[] PROGMEM = "1-";
const char string84[] PROGMEM = "abs";
const char string85[] PROGMEM = "random";
const char string86[] PROGMEM = "max";
const char string87[] PROGMEM = "min";
const char string88[] PROGMEM = "=";
const char string89[] PROGMEM = "<";
const char string90[] PROGMEM = "<=";
const char string91[] PROGMEM = ">";
const char string92[] PROGMEM = ">=";
const char string93[] PROGMEM = "/=";
const char string94[] PROGMEM = "plusp";
const char string95[] PROGMEM = "minusp";
const char string96[] PROGMEM = "zerop";
const char string97[] PROGMEM = "oddp";
const char string98[] PROGMEM = "evenp";
const char string99[] PROGMEM = "char";
const char string100[] PROGMEM = "char-code";
const char string101[] PROGMEM = "code-char";
const char string102[] PROGMEM = "characterp";
const char string103[] PROGMEM = "stringp";
const char string104[] PROGMEM = "string=";
const char string105[] PROGMEM = "string<";
const char string106[] PROGMEM = "string>";
const char string107[] PROGMEM = "sort";
const char string108[] PROGMEM = "string";
const char string109[] PROGMEM = "concatenate";
const char string110[] PROGMEM = "subseq";
const char string111[] PROGMEM = "read-from-string";
const char string112[] PROGMEM = "princ-to-string";
const char string113[] PROGMEM = "prin1-to-string";
const char string114[] PROGMEM = "logand";
const char string115[] PROGMEM = "logior";
const char string116[] PROGMEM = "logxor";
const char string117[] PROGMEM = "lognot";
const char string118[] PROGMEM = "ash";
const char string119[] PROGMEM = "logbitp";
const char string120[] PROGMEM = "eval";
const char string121[] PROGMEM = "globals";
const char string122[] PROGMEM = "locals";
const char string123[] PROGMEM = "makunbound";
const char string124[] PROGMEM = "break";
const char string125[] PROGMEM = "read";
const char string126[] PROGMEM = "prin1";
const char string127[] PROGMEM = "print";
const char string128[] PROGMEM = "princ";
const char string129[] PROGMEM = "terpri";
const char string130[] PROGMEM = "read-byte";
const char string131[] PROGMEM = "read-line";
const char string132[] PROGMEM = "write-byte";
const char string133[] PROGMEM = "write-string";
const char string134[] PROGMEM = "write-line";
const char string135[] PROGMEM = "restart-i2c";
const char string136[] PROGMEM = "gc";
const char string137[] PROGMEM = "room";
const char string138[] PROGMEM = "save-image";
const char string139[] PROGMEM = "load-image";
const char string140[] PROGMEM = "cls";
const char string141[] PROGMEM = "pinmode";
const char string142[] PROGMEM = "digitalread";
const char string143[] PROGMEM = "digitalwrite";
const char string144[] PROGMEM = "analogread";
const char string145[] PROGMEM = "analogwrite";
const char string146[] PROGMEM = "delay";
const char string147[] PROGMEM = "millis";
const char string148[] PROGMEM = "note";
const char string149[] PROGMEM = "edit";
const char string150[] PROGMEM = "pprint";

const tbl_entry_t lookup_table[] PROGMEM = {
  { string0, NULL, NIL, NIL },
  { string1, NULL, 0, 0 },
  { string2, NULL, 1, 0 },
  { string3, NULL, 1, 0 },
  { string4, NULL, 1, 0 },
  { string5, NULL, 0, 127 },
  { string6, NULL, 0, 127 },
  { string7, NULL, 0, 127 },
  { string8, NULL, 0, 127 },
  { string9, NULL, NIL, NIL },
  { string10, sp_quote, 1, 1 },
  { string11, sp_defun, 0, 127 },
  { string12, sp_defvar, 2, 2 },
  { string13, sp_setq, 2, 2 },
  { string14, sp_loop, 0, 127 },
  { string15, sp_push, 2, 2 },
  { string16, sp_pop, 1, 1 },
  { string17, sp_incf, 1, 2 },
  { string18, sp_decf, 1, 2 },
  { string19, sp_setf, 2, 2 },
  { string20, sp_dolist, 1, 127 },
  { string21, sp_dotimes, 1, 127 },
  { string22, sp_trace, 0, 1 },
  { string23, sp_untrace, 0, 1 },
  { string24, sp_formillis, 1, 127 },
  { string25, sp_withserial, 1, 127 },
  { string26, sp_withi2c, 1, 127 },
  { string27, sp_withspi, 1, 127 },
  { string28, NULL, NIL, NIL },
  { string29, tf_progn, 0, 127 },
  { string30, tf_return, 0, 127 },
  { string31, tf_if, 2, 3 },
  { string32, tf_cond, 0, 127 },
  { string33, tf_when, 1, 127 },
  { string34, tf_unless, 1, 127 },
  { string35, tf_and, 0, 127 },
  { string36, tf_or, 0, 127 },
  { string37, NULL, NIL, NIL },
  { string38, fn_not, 1, 1 },
  { string39, fn_not, 1, 1 },
  { string40, fn_cons, 2, 2 },
  { string41, fn_atom, 1, 1 },
  { string42, fn_listp, 1, 1 },
  { string43, fn_consp, 1, 1 },
  { string44, fn_numberp, 1, 1 },
  { string45, fn_symbolp, 1, 1 },
  { string46, fn_streamp, 1, 1 },
  { string47, fn_eq, 2, 2 },
  { string48, fn_car, 1, 1 },
  { string49, fn_car, 1, 1 },
  { string50, fn_cdr, 1, 1 },
  { string51, fn_cdr, 1, 1 },
  { string52, fn_caar, 1, 1 },
  { string53, fn_cadr, 1, 1 },
  { string54, fn_cadr, 1, 1 },
  { string55, fn_cdar, 1, 1 },
  { string56, fn_cddr, 1, 1 },
  { string57, fn_caaar, 1, 1 },
  { string58, fn_caadr, 1, 1 },
  { string59, fn_cadar, 1, 1 },
  { string60, fn_caddr, 1, 1 },
  { string61, fn_caddr, 1, 1 },
  { string62, fn_cdaar, 1, 1 },
  { string63, fn_cdadr, 1, 1 },
  { string64, fn_cddar, 1, 1 },
  { string65, fn_cdddr, 1, 1 },
  { string66, fn_length, 1, 1 },
  { string67, fn_list, 0, 127 },
  { string68, fn_reverse, 1, 1 },
  { string69, fn_nth, 2, 2 },
  { string70, fn_assoc, 2, 2 },
  { string71, fn_member, 2, 2 },
  { string72, fn_apply, 2, 127 },
  { string73, fn_funcall, 1, 127 },
  { string74, fn_append, 0, 127 },
  { string75, fn_mapc, 2, 3 },
  { string76, fn_mapcar, 2, 3 },
  { string77, fn_add, 0, 127 },
  { string78, fn_subtract, 1, 127 },
  { string79, fn_multiply, 0, 127 },
  { string80, fn_divide, 2, 127 },
  { string81, fn_mod, 2, 2 },
  { string82, fn_oneplus, 1, 1 },
  { string83, fn_oneminus, 1, 1 },
  { string84, fn_abs, 1, 1 },
  { string85, fn_random, 1, 1 },
  { string86, fn_max, 1, 127 },
  { string87, fn_min, 1, 127 },
  { string88, fn_numeq, 1, 127 },
  { string89, fn_less, 1, 127 },
  { string90, fn_lesseq, 1, 127 },
  { string91, fn_greater, 1, 127 },
  { string92, fn_greatereq, 1, 127 },
  { string93, fn_noteq, 1, 127 },
  { string94, fn_plusp, 1, 1 },
  { string95, fn_minusp, 1, 1 },
  { string96, fn_zerop, 1, 1 },
  { string97, fn_oddp, 1, 1 },
  { string98, fn_evenp, 1, 1 },
  { string99, fn_char, 2, 2 },
  { string100, fn_charcode, 1, 1 },
  { string101, fn_codechar, 1, 1 },
  { string102, fn_characterp, 1, 1 },
  { string103, fn_stringp, 1, 1 },
  { string104, fn_stringeq, 2, 2 },
  { string105, fn_stringless, 2, 2 },
  { string106, fn_stringgreater, 2, 2 },
  { string107, fn_sort, 2, 2 },
  { string108, fn_stringfn, 1, 1 },
  { string109, fn_concatenate, 1, 127 },
  { string110, fn_subseq, 2, 3 },
  { string111, fn_readfromstring, 1, 1 },
  { string112, fn_princtostring, 1, 1 },
  { string113, fn_prin1tostring, 1, 1 },
  { string114, fn_logand, 0, 127 },
  { string115, fn_logior, 0, 127 },
  { string116, fn_logxor, 0, 127 },
  { string117, fn_lognot, 1, 1 },
  { string118, fn_ash, 2, 2 },
  { string119, fn_logbitp, 2, 2 },
  { string120, fn_eval, 1, 1 },
  { string121, fn_globals, 0, 0 },
  { string122, fn_locals, 0, 0 },
  { string123, fn_makunbound, 1, 1 },
  { string124, fn_break, 0, 0 },
  { string125, fn_read, 0, 0 },
  { string126, fn_prin1, 1, 1 },
  { string127, fn_print, 1, 1 },
  { string128, fn_princ, 1, 1 },
  { string129, fn_terpri, 0, 0 },
  { string130, fn_readbyte, 0, 2 },
  { string131, fn_readline, 0, 1 },
  { string132, fn_writebyte, 1, 2 },
  { string133, fn_writestring, 1, 2 },
  { string134, fn_writeline, 1, 2 },
  { string135, fn_restarti2c, 1, 2 },
  { string136, fn_gc, 0, 0 },
  { string137, fn_room, 0, 0 },
  { string138, fn_saveimage, 0, 1 },
  { string139, fn_loadimage, 0, 0 },
  { string140, fn_cls, 0, 0 },
  { string141, fn_pinmode, 2, 2 },
  { string142, fn_digitalread, 1, 1 },
  { string143, fn_digitalwrite, 2, 2 },
  { string144, fn_analogread, 1, 1 },
  { string145, fn_analogwrite, 2, 2 },
  { string146, fn_delay, 1, 1 },
  { string147, fn_millis, 0, 0 },
  { string148, fn_note, 0, 3 },
  { string149, fn_edit, 1, 1 },
  { string150, fn_pprint, 1, 1 },
};

// Table lookup functions

int builtin (char* n) {
  int entry = 0;
  while (entry < ENDFUNCTIONS) {
    if(strcmp_P(n, (char*)pgm_read_word(&lookup_table[entry].string)) == 0)
      return entry;
    entry++;
  }
  return ENDFUNCTIONS;
}

int longsymbol (char *buffer) {
  char *p = SymbolTable;
  int i = 0;
  while (strcmp(p, buffer) != 0) {p = p + strlen(p) + 1; i++; }
  if (p == buffer) {
    // Add to symbol table?
    char *newtop = SymbolTop + strlen(p) + 1;
    if (SYMBOLTABLESIZE - (newtop - SymbolTable) < BUFFERSIZE) error(PSTR("No room for long symbols"));
    SymbolTop = newtop;
  }
  if (i > 1535) error(PSTR("Too many long symbols"));
  return i + 64000; // First number unused by radix40
}

int lookupfn (symbol_t name) {
  return pgm_read_word(&lookup_table[name].fptr);
}

int lookupmin (symbol_t name) {
  return pgm_read_word(&lookup_table[name].min);
}

int lookupmax (symbol_t name) {
  return pgm_read_word(&lookup_table[name].max);
}

char *lookupbuiltin (symbol_t name) {
  char *buffer = SymbolTop;
  strcpy_P(buffer, (char *)(pgm_read_word(&lookup_table[name].string)));
  return buffer;
}

char *lookupsymbol (symbol_t name) {
  char *p = SymbolTable;
  int i = name - 64000;
  while (i > 0 && p < SymbolTop) {p = p + strlen(p) + 1; i--; }
  if (p == SymbolTop) return NULL; else return p;
}

void deletesymbol (symbol_t name) {
  char *p = lookupsymbol(name);
  if (p == NULL) return;
  char *q = p + strlen(p) + 1;
  *p = '\0'; p++;
  while (q < SymbolTop) *(p++) = *(q++);
  SymbolTop = p;
}

void testescape () {
  if (Serial.read() == '~') error(PSTR("Escape!"));
}

// Main evaluator

object *eval (object *form, object *env) {
  int TC=0;
  EVAL:
  // Enough space?
  if (Freespace < 20) gc(form, env);
  if (_end != 0xA5) error(PSTR("Stack overflow"));
  // Escape
  if (tstflag(ESCAPE)) { clrflag(ESCAPE); error(PSTR("Escape!"));}
  #if defined (serialmonitor)
  testescape();
  #endif 
  
  if (form == NULL) return nil;

  if (numberp(form) || characterp(form) || stringp(form)) return form;

  if (symbolp(form)) {
    symbol_t name = form->name;
    if (name == NIL) return nil;
    object *pair = value(name, env);
    if (pair != NULL) return cdr(pair);
    pair = value(name, GlobalEnv);
    if (pair != NULL) return cdr(pair);
    else if (name <= ENDFUNCTIONS) return form;
    error2(form, PSTR("undefined"));
  }
  
  // It's a list
  object *function = car(form);
  object *args = cdr(form);
  if (!listp(args)) error(PSTR("Can't evaluate a dotted pair"));

  // List starts with a symbol?
  if (symbolp(function)) {
    symbol_t name = function->name;

    if ((name == LET) || (name == LETSTAR)) {
      int TCstart = TC;
      object *assigns = first(args);
      object *forms = cdr(args);
      object *newenv = env;
      push(newenv, GCStack);
      while (assigns != NULL) {
        object *assign = car(assigns);
        if (consp(assign)) push(cons(first(assign),eval(second(assign),env)), newenv);
        else push(cons(assign,nil), newenv);
        car(GCStack) = newenv;
        if (name == LETSTAR) env = newenv;
        assigns = cdr(assigns);
      }
      env = newenv;
      pop(GCStack);
      form = tf_progn(forms,env);
      TC = TCstart;
      goto EVAL;
    }

    if (name == LAMBDA) {
      if (env == NULL) return form;
      object *envcopy = NULL;
      while (env != NULL) {
        object *pair = first(env);
        if (pair != NULL) {
          object *val = cdr(pair);
          if (numberp(val)) val = number(val->integer);
          push(cons(car(pair), val), envcopy);
        }
        env = cdr(env);
      }
      return cons(symbol(CLOSURE), cons(envcopy,args));
    }
    
    if ((name > SPECIAL_FORMS) && (name < TAIL_FORMS)) {
      return ((fn_ptr_type)lookupfn(name))(args, env);
    }

    if ((name > TAIL_FORMS) && (name < FUNCTIONS)) {
      form = ((fn_ptr_type)lookupfn(name))(args, env);
      TC = 1;
      goto EVAL;
    }
  }
        
  // Evaluate the parameters - result in head
  object *fname = car(form);
  int TCstart = TC;
  object *head = cons(eval(car(form), env), NULL);
  push(head, GCStack); // Don't GC the result list
  object *tail = head;
  form = cdr(form);
  int nargs = 0;

  while (form != NULL){
    object *obj = cons(eval(car(form),env),NULL);
    cdr(tail) = obj;
    tail = obj;
    form = cdr(form);
    nargs++;
  }
    
  function = car(head);
  args = cdr(head);
 
  if (symbolp(function)) {
    symbol_t name = function->name;
    if (name >= ENDFUNCTIONS) error2(fname, PSTR("is not valid here"));
    if (nargs<lookupmin(name)) error2(fname, PSTR("has too few arguments"));
    if (nargs>lookupmax(name)) error2(fname, PSTR("has too many arguments"));
    object *result = ((fn_ptr_type)lookupfn(name))(args, env);
    pop(GCStack);
    return result;
  }
      
  if (listp(function) && issymbol(car(function), LAMBDA)) {
    dropframe(TCstart, &env);
    form = closure(fname, NULL, cdr(function), args, &env);
    pop(GCStack);
    int trace = tracing(fname->name);
    if (trace) {
      object *result = eval(form, env);
      indent((--(TraceDepth[trace-1]))<<1);
      pint(TraceDepth[trace-1], pserial);
      pserial(':'); pserial(' ');
      printobject(fname, pserial); pfstring(PSTR(" returned "), pserial);
      printobject(result, pserial); pln(pserial);
      return result;
    } else {
      TC = 1;
      goto EVAL;
    }
  }

  if (listp(function) && issymbol(car(function), CLOSURE)) {
    function = cdr(function);
    dropframe(TCstart, &env);
    form = closure(fname, car(function), cdr(function), args, &env);
    pop(GCStack);
    TC = 1;
    goto EVAL;
  } 
  
  error2(fname, PSTR("is an illegal function")); return nil;
}

// Print functions

void pserial (char c) {
  LastPrint = c;
  if (c == '\n') Serial.write('\r');
  Serial.write(c);
}

const char ControlCodes[] PROGMEM = "Null\0SOH\0STX\0ETX\0EOT\0ENQ\0ACK\0Bell\0Backspace\0Tab\0Newline\0VT\0"
"Page\0Return\0SO\0SI\0DLE\0DC1\0DC2\0DC3\0DC4\0NAK\0SYN\0ETB\0CAN\0EM\0SUB\0Escape\0FS\0GS\0RS\0US\0Space\0";

void pcharacter (char c, pfun_t pfun) {
  if (!PrintReadably) pfun(c);
  else {
    pfun('#'); pfun('\\');
    if (c > 32) pfun(c);
    else {
      PGM_P p = ControlCodes;
      while (c > 0) {p = p + strlen_P(p) + 1; c--; }
      pfstring(p, pfun);
    }
  }
}

void pstring (char *s, pfun_t pfun) {
  while (*s) pfun(*s++);
}

void printstring (object *form, pfun_t pfun) {
  if (PrintReadably) pfun('"');
  form = cdr(form);
  while (form != NULL) {
    int chars = form->integer;
    char ch = chars>>8 & 0xFF;
    if (PrintReadably && (ch == '"' || ch == '\\')) pfun('\\');
    pfun(ch);
    ch = chars & 0xFF;
    if (PrintReadably && (ch == '"' || ch == '\\')) pfun('\\');
    if (ch) pfun(ch);
    form = car(form);
  }
  if (PrintReadably) pfun('"');
}

void pfstring (PGM_P s, pfun_t pfun) {
  int p = (int)s;
  while (1) {
    char c = pgm_read_byte(p++);
    if (c == 0) return;
    pfun(c);
  }
}

void pint (int i, pfun_t pfun) {
  int lead = 0;
  if (i<0) pfun('-');
  for (int d=10000; d>0; d=d/10) {
    int j = i/d;
    if (j!=0 || lead || d==1) { pfun(abs(j)+'0'); lead=1;}
    i = i - j*d;
  }
}

inline void pln (pfun_t pfun) {
  pfun('\n');
}

void pfl (pfun_t pfun) {
  if (LastPrint != '\n') pfun('\n');
}

void printobject(object *form, pfun_t pfun){
  if (form == NULL) pfstring(PSTR("nil"), pfun);
  else if (listp(form) && issymbol(car(form), CLOSURE)) pfstring(PSTR("<closure>"), pfun);
  else if (listp(form)) {
    pfun('(');
    printobject(car(form), pfun);
    form = cdr(form);
    while (form != NULL && listp(form)) {
      pfun(' ');
      printobject(car(form), pfun);
      form = cdr(form);
    }
    if (form != NULL) {
      pfstring(PSTR(" . "), pfun);
      printobject(form, pfun);
    }
    pfun(')');
  } else if (numberp(form)) {
    pint(integer(form), pfun);
  } else if (symbolp(form)) {
    if (form->name != NOTHING) pstring(name(form), pfun);
  } else if (characterp(form)) {
    pcharacter(form->integer, pfun);
  } else if (stringp(form)) {
    printstring(form, pfun);
  } else if (streamp(form)) {
    pfstring(PSTR("<"), pfun);
    if ((form->integer)>>8 == SPISTREAM) pfstring(PSTR("spi"), pfun);
    else if ((form->integer)>>8 == I2CSTREAM) pfstring(PSTR("i2c"), pfun);
    else pfstring(PSTR("serial"), pfun);
    pfstring(PSTR("-stream "), pfun);
    pint(form->integer & 0xFF, pfun);
    pfun('>');
  } else
    error(PSTR("Error in print."));
}

// Read functions

char gserial () {
  if (LastChar) { 
    char temp = LastChar;
    LastChar = 0;
    return temp;
  }
  while (!Serial.available());
  char temp = Serial.read();
  if (temp != '\n') pserial(temp);
  return temp;
}

object *nextitem (gfun_t gfun) {
  char ch = gfun();
  while(isspace(ch)) ch = gfun();

  if (ch == ';') {
    while(ch != '(') ch = gfun();
    ch = '(';
  }
  if (ch == '\n') ch = gfun();
  if (ch == EOF) exit(0);

  if (ch == ')') return (object *)KET;
  if (ch == '(') return (object *)BRA;
  if (ch == '\'') return (object *)QUO;
  if (ch == '.') return (object *)DOT;

  // Parse string
  if (ch == '"') return readstring('"', gfun);
  
  // Parse variable, character, or number
  int index = 0, base = 10, sign = 1;
  char *buffer = SymbolTop;
  int bufmax = SYMBOLTABLESIZE-(buffer-SymbolTable)-1; // Max index
  unsigned int result = 0;
  if (ch == '+') {
    buffer[index++] = ch;
    ch = gfun();
  } else if (ch == '-') {
    sign = -1;
    buffer[index++] = ch;
    ch = gfun();
  } else if (ch == '#') {
    ch = gfun() & ~0x20;
    if (ch == '\\') base = 0; // character
    else if (ch == 'B') base = 2;
    else if (ch == 'O') base = 8;
    else if (ch == 'X') base = 16;
    else error(PSTR("Illegal character after #"));
    ch = gfun();
  }
  int isnumber = (digitvalue(ch)<base);
  buffer[2] = '\0'; // In case variable is one letter

  while(!isspace(ch) && ch != ')' && ch != '(' && index < bufmax) {
    buffer[index++] = ch;
    int temp = digitvalue(ch);
    result = result * base + temp;
    isnumber = isnumber && (digitvalue(ch)<base);
    ch = gfun();
  }

  buffer[index] = '\0';
  if (ch == ')') LastChar = ')';
  if (ch == '(') LastChar = '(';

  if (isnumber) {
    if (base == 10 && result > ((unsigned int)32767+(1-sign)/2)) error(PSTR("Number out of range"));
    return number(result*sign);
  } else if (base == 0) {
    if (index == 1) return character(buffer[0]);
    PGM_P p = ControlCodes; char c = 0;
    while (c < 33) {
      if (strcasecmp_P(buffer, p) == 0) return character(c);
      p = p + strlen_P(p) + 1; c++;
    }
    error(PSTR("Unknown character"));
  }
  
  int x = builtin(buffer);
  if (x == NIL) return nil;
  if (x < ENDFUNCTIONS) return symbol(x);
  else if (index < 4 && valid40(buffer)) return symbol(pack40(buffer));
  else return symbol(longsymbol(buffer));
}

object *readrest (gfun_t gfun) {
  object *item = nextitem(gfun);
  object *head = NULL;
  object *tail = NULL;

  while (item != (object *)KET) {
    if (item == (object *)BRA) {
      item = readrest(gfun);
    } else if (item == (object *)QUO) {
      item = cons(symbol(QUOTE), cons(read(gfun), NULL));
    } else if (item == (object *)DOT) {
      tail->cdr = read(gfun);
      if (readrest(gfun) != NULL) error(PSTR("Malformed list"));
      return head;
    } else {
      object *cell = cons(item, NULL);
      if (head == NULL) head = cell;
      else tail->cdr = cell;
      tail = cell;
      item = nextitem(gfun);
    }
  }
  return head;
}

object *read (gfun_t gfun) {
  object *item = nextitem(gfun);
  if (item == (object *)BRA) return readrest(gfun);
  if (item == (object *)DOT) return read(gfun);
  if (item == (object *)QUO) return cons(symbol(QUOTE), cons(read(gfun), NULL)); 
  return item;
}

// Setup

void initenv() {
  GlobalEnv = NULL;
  tee = symbol(TEE);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);  // wait for Serial to initialize
  initworkspace();
  initenv();
  _end = 0xA5;      // Canary to check stack
  pfstring(PSTR("uLisp 1.9 "), pserial); pln(pserial);
}

// Read/Evaluate/Print loop

void repl (object *env) {
  for (;;) {
    randomSeed(micros());
    gc(NULL, env);
    #if defined (printfreespace)
    pint(Freespace, pserial);
    #endif
    if (BreakLevel) {
      pfstring(PSTR(" : "), pserial);
      pint(BreakLevel, pserial);
    }
    pfstring(PSTR("> "), pserial);
    object *line = read(gserial);
    if (BreakLevel && line == nil) { pln(pserial); return; }
    if (line == (object *)KET) error(PSTR("Unmatched right bracket"));
    push(line, GCStack);
    pfl(pserial);
    line = eval(line, env);
    pfl(pserial);
    printobject(line, pserial);
    pop(GCStack);
    pfl(pserial);
    pln(pserial);
  }
}

void loop () {
  if (!setjmp(exception)) {
    #if defined(resetautorun)
    autorunimage();
    #endif
  }
  repl(NULL);
}
