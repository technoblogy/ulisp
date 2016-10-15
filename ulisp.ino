/* uLisp Version 1.1 - www.ulisp.com
   Copyright (c) 2016 David Johnson-Davies
   
   Licensed under the MIT license: https://opensource.org/licenses/MIT
*/

#include <setjmp.h>

// Compile options

#define checkoverflow
#define resetautorun

// C Macros

#define nil                NULL
#define car(x)             (((object *) (x))->car)
#define cdr(x)             (((object *) (x))->cdr)

#define first(x)           (((object *) (x))->car)
#define second(x)          (car(cdr(x)))
#define third(x)           (car(cdr(cdr(x))))
#define fourth(x)          (car(cdr(cdr(cdr(x)))))

#define push(x, y)         ((y) = cons((x),(y)))
#define pop(y)             ((y) = cdr(y))

#define numberp(x)         ((x)->type == NUMBER)
#define listp(x)           ((x)->type != NUMBER && (x)->type != SYMBOL)
#define consp(x)           ((x)->type != NUMBER && (x)->type != SYMBOL && (x) != NULL)

#define mark(x)            (car(x) = (object *)(((unsigned int)(car(x))) | 0x8000))
#define unmark(x)          (car(x) = (object *)(((unsigned int)(car(x))) & 0x7FFF))
#define marked(x)          ((((unsigned int)(car(x))) & 0x8000) != 0)

// 1:Show GCs 2:show symbol addresses
// #define debug1
// #define debug2

// Constants

#if defined(__AVR_ATmega328P__)
const int workspacesize = 317;
const int EEPROMsize = 1024;
#elif defined(__AVR_ATmega32U4__)
const int workspacesize = 421;
const int EEPROMsize = 1024;
#elif defined(__AVR_ATmega2560__)
const int workspacesize = 1461;
const int EEPROMsize = 4096;
#elif defined(__AVR_ATmega1284P__)
const int workspacesize = 3000;
const int EEPROMsize = 4096;
#else
const int workspacesize = 317;
const int EEPROMsize = 1024;
#endif

const int buflen = 17;  // Length of longest symbol + 1
enum type {NONE, SYMBOL, NUMBER};
enum token { UNUSED, BRA, KET, QUO, DOT};

enum function { SYMBOLS, NIL, TEE, LAMBDA, LET, LETSTAR, CLOSURE, SPECIAL_FORMS, QUOTE, DEFUN, DEFVAR, SETQ, LOOP, PUSH, POP, INCF,
DECF, DOLIST, DOTIMES, FORMILLIS, TAIL_FORMS, PROGN, RETURN, IF, COND, WHEN, UNLESS, AND, OR, FUNCTIONS, NOT, NULLFN, CONS, ATOM, 
LISTP, CONSP, NUMBERP, EQ, CAR, FIRST, CDR, REST, CAAR, CADR, SECOND, CDAR, CDDR, CAAAR, CAADR, CADAR, CADDR, THIRD, CDAAR, CDADR, 
CDDAR, CDDDR, LENGTH, LIST, REVERSE, NTH, ASSOC, MEMBER, APPLY, FUNCALL, APPEND, MAPC, MAPCAR, ADD, SUBTRACT, MULTIPLY, DIVIDE, MOD,
ONEPLUS, ONEMINUS, ABS, RANDOM, MAX, MIN, NUMEQ, LESS, LESSEQ, GREATER, GREATEREQ, NOTEQ, PLUSP, MINUSP, ZEROP, ODDP, EVENP, READ,
EVAL, LOCALS, GLOBALS, MAKUNBOUND, BREAK, PRINT, PRINC, GC, SAVEIMAGE, LOADIMAGE, PINMODE, DIGITALREAD, DIGITALWRITE, ANALOGREAD, 
ANALOGWRITE, DELAY, MILLIS, SHIFTOUT, SHIFTIN, NOTE, ENDFUNCTIONS };

// Typedefs

typedef struct sobject {
  union {
    struct {
      sobject *car;
      sobject *cdr;
    };
    struct {
      enum type type;
      union {
        unsigned int name;
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

// Global variables

jmp_buf exception;
object workspace[workspacesize];
unsigned int freespace = 0;
char ReturnFlag = 0;
object *freelist;
extern uint8_t _end;

object *GlobalEnv;
object *GCStack = NULL;
char buffer[buflen+1];
char BreakLevel = 0;
char LastChar = 0;

// Forward references
object *tee;
object *tf_progn (object *form, object *env);
object *eval (object *form, object *env);
object *read();
void repl();
void printobject (object *form);
char *lookupstring (unsigned int name);
int lookupfn(unsigned int name);
int builtin(char* n);

// Set up workspace

void initworkspace () {
  freelist = NULL;
  for (int i=workspacesize-1; i>=0; i--) {
    object *obj = &workspace[i];
    car(obj) = NULL;
    cdr(obj) = freelist;
    freelist = obj;
    freespace++;
  }
}

object *myalloc() {
  if (freespace == 0) error(F("No room"));
  object *temp = freelist;
  freelist = cdr(freelist);
  freespace--;
  return temp;
}

void myfree (object *obj) {
  cdr(obj) = freelist;
  freelist = obj;
  freespace++;
}

// Make each type of object

object *number (int n) {
  object *ptr = (object *) myalloc ();
  ptr->type = NUMBER;
  ptr->integer = n;
  return ptr;
}

object *cons (object *arg1, object *arg2) {
  object *ptr = (object *) myalloc ();
  ptr->car = arg1;
  ptr->cdr = arg2;
  return ptr;
}

object *symbol (unsigned int name) {
  object *ptr = (object *) myalloc ();
  ptr->type = SYMBOL;
  ptr->name = name;
  return ptr;
}

// Garbage collection

void markobject (object *obj) {
  MARK:
  if (obj == NULL) return;
  
  object* arg = car(obj);
  if (marked(obj)) return;

  int type = obj->type;
  mark(obj);
  
  if (type != SYMBOL && type != NUMBER) { // cons
    markobject(arg);
    obj = cdr(obj);
    goto MARK;
  }
}

void sweep () {
  freelist = NULL;
  freespace = 0;
  for (int i=workspacesize-1; i>=0; i--) {
    object *obj = &workspace[i];
    if (!marked(obj)) {
      car(obj) = NULL;
      cdr(obj) = freelist;
      freelist = obj;
      freespace++;
    } else unmark(obj);
  }
}

void gc (object *form, object *env) {
  #if defined(debug1)
  int start = freespace;
  #endif
  markobject(tee); 
  markobject(GlobalEnv);
  markobject(GCStack);
  markobject(form);
  markobject(env);
  sweep();
  #if defined(debug1)
  Serial.print('{');
  Serial.print(freespace - start);
  Serial.println('}');
  #endif
}

// Save-image and load-image

typedef struct {
  unsigned int eval;
  unsigned int datasize;
  unsigned int globalenv;
  unsigned int tee;
  char data[];
} struct_image;

struct_image EEMEM image;

void movepointer (object *from, object *to) {
  for (int i=0; i<workspacesize; i++) {
    object *obj = &workspace[i];
    int type = (obj->type) & 0x7FFF;
    if (marked(obj) && type != SYMBOL && type != NUMBER) {
      if (car(obj) == (object *)((unsigned int)from | 0x8000)) 
        car(obj) = (object *)((unsigned int)to | 0x8000);
      if (cdr(obj) == from) cdr(obj) = to;
    }
  }
}
  
int compactimage (object **arg) {
  markobject(tee);
  markobject(GlobalEnv);
  markobject(GCStack);
  object *firstfree = workspace;
  while (marked(firstfree)) firstfree++;

  for (int i=0; i<workspacesize; i++) {
    object *obj = &workspace[i];
    if (marked(obj) && firstfree < obj) {
      car(firstfree) = car(obj);
      cdr(firstfree) = cdr(obj);
      unmark(obj);
      movepointer(obj, firstfree);
      if (GlobalEnv == obj) GlobalEnv = firstfree;
      if (GCStack == obj) GCStack = firstfree;
      if (tee == obj) tee = firstfree;
      if (*arg == obj) *arg = firstfree;
      while (marked(firstfree)) firstfree++;
    }
  }
  sweep();
  return firstfree - workspace;
}
  
int saveimage (object *arg) {
  unsigned int imagesize = compactimage(&arg);
  // Save to EEPROM
  if ((imagesize*4+8) > EEPROMsize) {
    Serial.print(F("Error: Image size too large: "));
    Serial.println(imagesize+2);
    GCStack = NULL;
    longjmp(exception, 1);
  }
  eeprom_write_word(&image.datasize, imagesize);
  eeprom_write_word(&image.eval, (unsigned int)arg);
  eeprom_write_word(&image.globalenv, (unsigned int)GlobalEnv);
  eeprom_write_word(&image.tee, (unsigned int)tee);
  eeprom_write_block(workspace, image.data, imagesize*4);
  return imagesize+2;
}

int loadimage () {
  unsigned int imagesize = eeprom_read_word(&image.datasize);
  if (imagesize == 0 || imagesize == 0xFFFF) error(F("No saved image"));
  GlobalEnv = (object *)eeprom_read_word(&image.globalenv);
  tee = (object *)eeprom_read_word(&image.tee) ;
  eeprom_read_block(workspace, image.data, imagesize*4);
  gc(NULL, NULL);
  return imagesize+2;
}

// Error handling

void error (const __FlashStringHelper *string) {
  Serial.print(F("Error: "));
  Serial.println(string);
  GCStack = NULL;
  longjmp(exception, 1);
}

void error2 (object *symbol, const __FlashStringHelper *string) {
  Serial.print(F("Error: '"));
  printobject(symbol);
  Serial.print("' ");
  Serial.println(string);
  GCStack = NULL;
  longjmp(exception, 1);
}

// Helper functions

int toradix40 (int ch) {
  if (ch == 0) return 0;
  if (ch >= '0' && ch <= '9') return ch-'0'+30;
  ch = ch | 0x20;
  if (ch >= 'a' && ch <= 'z') return ch-'a'+1;
  error(F("Illegal character in symbol"));
  return 0;
}

int fromradix40 (int n) {
  if (n >= 1 && n <= 26) return 'a'+n-1;
  if (n >= 30 && n <= 39) return '0'+n-30;
  if (n == 27) return '-';
  return 0;
}

int pack40 (char *buffer) {
  return (((toradix40(buffer[0]) * 40) + toradix40(buffer[1])) * 40 + toradix40(buffer[2]));
}

int digitvalue (char d) {
  if (d>='0' && d<='9') return d-'0';
  d = d | 0x20;
  if (d>='a' && d<='f') return d-'a'+10;
  return 16;
}

char *name(object *obj){
  buffer[3] = '\0';
  if(obj->type != SYMBOL) error(F("Error in name"));
  unsigned int x = obj->name;
  if (x < ENDFUNCTIONS) return lookupstring(x);
  for (int n=2; n>=0; n--) {
    buffer[n] = fromradix40(x % 40);
    x = x / 40;
  }
  return buffer;
}

int integer(object *obj){
  if(obj->type != NUMBER) error(F("not a number"));
  return obj->integer;
}

int issymbol(object *obj, unsigned int n) {
  return obj->type == SYMBOL && obj->name == n;
}

int eq (object *arg1, object *arg2) {
  int same_object = (arg1 == arg2);
  int same_symbol = (arg1->type == SYMBOL && arg2->type == SYMBOL && arg1->name == arg2->name);
  int same_number = (arg1->type == NUMBER && arg2->type == NUMBER && arg1->integer == arg2->integer);
  return (same_object || same_symbol || same_number);
}

// Lookup variable in environment

object *value(unsigned int n, object *env) {
  while (env != NULL) {
    object *item = car(env);
    if(car(item)->name == n) return item;
    env = cdr(env);
  }
  return nil;
}

object *findvalue (object *var, object *env) {
  unsigned int varname = var->name;
  object *pair = value(varname, env);
  if (pair == NULL) pair = value(varname, GlobalEnv);
  if (pair == NULL) error2(var,F("unknown variable"));
  return pair;
}

object *findtwin (object *var, object *env) {
  while (env != NULL) {
    object *pair = car(env);
    if (car(pair) == var) return pair;
    env = cdr(env);
  }
  return NULL;
}

object *closure (int tail, object *fname, object *state, object *function, object *args, object **env) {
  object *params = first(function);
  function = cdr(function);
  // Push state if not already in env
  while (state != NULL) {
    object *pair = first(state);
    if (findtwin(car(pair), *env) == NULL) push(first(state), *env);
    state = cdr(state);
  }
  // Add arguments to environment
  while (params != NULL && args != NULL) {
    object *var = first(params);
    object *value = first(args);
    if (tail) {
      object *pair = findtwin(var, *env);
      if (pair != NULL) cdr(pair) = value;
      else push(cons(var,value), *env);
    } else push(cons(var,value), *env);
    params = cdr(params);
    args = cdr(args);
  }
  if (params != NULL) error2(fname, F("has too few parameters"));
  if (args != NULL) error2(fname, F("has too many parameters"));
  // Do an implicit progn
  return tf_progn(function, *env); 
}

inline int listlength (object *list) {
  int length = 0;
  while (list != NULL) {
    list = cdr(list);
    length++;
  }
  return length;
}
  
object *apply (object *function, object *args, object **env) {
  if (function->type == SYMBOL) {
    unsigned int name = function->name;
    int nargs = listlength(args);
    if (name >= ENDFUNCTIONS) error2(function, F("is not a function"));
    if (nargs<lookupmin(name)) error2(function, F("has too few arguments"));
    if (nargs>lookupmax(name)) error2(function, F("has too many arguments"));
    return ((fn_ptr_type)lookupfn(name))(args, *env);
  }
  if (listp(function) && issymbol(car(function), LAMBDA)) {
    function = cdr(function);
    object *result = closure(0, NULL, NULL, function, args, env);
    return eval(result, *env);
  }
  if (listp(function) && issymbol(car(function), CLOSURE)) {
    function = cdr(function);
    object *result = closure(0, NULL, car(function), cdr(function), args, env);
    return eval(result, *env);
  }
  error2(function, F("illegal function"));
  return NULL;
}

// Checked car and cdr

inline object *carx (object *arg) {
  if (!listp(arg)) error(F("Can't take car"));
  if (arg == nil) return nil;
  return car(arg);
}

inline object *cdrx (object *arg) {
  if (!listp(arg)) error(F("Can't take cdr"));
  if (arg == nil) return nil;
  return cdr(arg);
}

// Special forms

object *sp_quote (object *args, object *env) {
  (void) env;
  return first(args);
}

object *sp_defun (object *args, object *env) {
  (void) env;
  object *var = first(args);
  if (var->type != SYMBOL) error2(var, F("is not a symbol"));
  object *val = cons(symbol(LAMBDA), cdr(args));
  object *pair = value(var->name,GlobalEnv);
  if (pair != NULL) { cdr(pair) = val; return var; }
  push(cons(var, val), GlobalEnv);
  return var;
}

object *sp_defvar (object *args, object *env) {
  object *var = first(args);
  if (var->type != SYMBOL) error2(var, F("is not a symbol"));
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
  ReturnFlag = 0;
  object *start = args;
  for (;;) {
    args = start;
    while (args != NULL) {
      object *form = car(args);
      object *result = eval(form,env);
      if (ReturnFlag == 1) {
        ReturnFlag = 0;
        return result;
      }
      args = cdr(args);
    }
  }
}

object *sp_push (object *args, object *env) {
  object *item = eval(first(args), env);
  object *pair = findvalue(second(args), env);
  push(item,cdr(pair));
  return cdr(pair);
}

object *sp_pop (object *args, object *env) {
  object *pair = findvalue(first(args), env);
  object *result = car(cdr(pair));
  pop(cdr(pair));
  return result;
}

object *sp_incf (object *args, object *env) {
  object *var = first(args);
  object *pair = findvalue(var, env);
  int result = integer(eval(var, env));
  int temp = 1;
  if (cdr(args) != NULL) temp = integer(eval(second(args), env));
  #if defined(checkoverflow)
  if (temp < 1) { if (-32768 - temp > result) error(F("'incf' arithmetic overflow")); }
  else { if (32767 - temp < result) error(F("'incf' arithmetic overflow")); }
  #endif
  result = result + temp;
  var = number(result);
  cdr(pair) = var;
  return var;
}

object *sp_decf (object *args, object *env) {
  object *var = first(args);
  object *pair = findvalue(var, env);
  int result = integer(eval(var, env));
  int temp = 1;
  if (cdr(args) != NULL) temp = integer(eval(second(args), env));
  #if defined(checkoverflow)
  if (temp < 1) { if (32767 + temp < result) error(F("'decf' arithmetic overflow")); }
  else { if (-32768 + temp > result) error(F("'decf' arithmetic overflow")); }
  #endif
  result = result - temp;
  var = number(result);
  cdr(pair) = var;
  return var;
}

object *sp_dolist (object *args, object *env) {
  object *params = first(args);
  object *var = first(params);
  object *list = eval(second(params), env);
  if (!listp(list)) error(F("'dolist' argument is not a list"));
  object *pair = cons(var,nil);
  push(pair,env);
  object *result = third(params);
  object *forms = cdr(args);
  while (list != NULL) {
    cdr(pair) = first(list);
    list = cdr(list);
    eval(tf_progn(forms,env), env);
  }
  cdr(pair) = nil;
  return eval(result, env);
}

object *sp_dotimes (object *args, object *env) {
  object *params = first(args);
  object *var = first(params);
  int count = integer(eval(second(params), env));
  int index = 0;
  object *result = third(params);
  object *pair = cons(var,number(0));
  push(pair,env);
  object *forms = cdr(args);
  while (index < count) {
    cdr(pair) = number(index);
    index++;
    eval(tf_progn(forms,env), env);
  }
  cdr(pair) = number(index);
  return eval(result, env);
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

// Tail-recursive forms

object *tf_progn (object *args, object *env) {
  if (args == NULL) return nil;
  object *more = cdr(args);
  while (more != NULL) {
  eval(car(args), env);
    args = more;
    more = cdr(args);
  }
  return car(args);
}

object *tf_return (object *args, object *env) {
  ReturnFlag = 1;
  return tf_progn(args, env);
}

object *tf_if (object *args, object *env) {
  if (eval(first(args), env) != nil) return second(args);
  return third(args);
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
  if (first(args) == nil) return tee; else return nil;
}

object *fn_cons (object *args, object *env) {
  (void) env;
  return cons(first(args),second(args));
}

object *fn_atom (object *args, object *env) {
  (void) env;
  object *arg1 = first(args);
  if (consp(arg1)) return nil; else return tee;
}

object *fn_listp (object *args, object *env) {
  (void) env;
  object *arg1 = first(args);
  if (listp(arg1)) return tee; else return nil;
}

object *fn_consp (object *args, object *env) {
  (void) env;
  object *arg1 = first(args);
  if (consp(arg1)) return tee; else return nil;
}

object *fn_numberp (object *args, object *env) {
  (void) env;
  object *arg1 = first(args);
  if (numberp(arg1)) return tee; else return nil;
}

object *fn_eq (object *args, object *env) {
  (void) env;
  object *arg1 = first(args);
  object *arg2 = second(args);
  if(eq(arg1, arg2)) return tee; else return nil;
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
  object *list = first(args);
  if (!listp(list)) error(F("'length' argument is not a list"));
  return number(listlength(list));
}

object *fn_list (object *args, object *env) {
  (void) env;
  return args;
}

object *fn_reverse (object *args, object *env) {
  (void) env;
  object *list = first(args);
  if (!listp(list)) error(F("'reverse' argument is not a list"));
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
  if (!listp(list)) error(F("'nth' second argument is not a list"));
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
  if (!listp(list)) error(F("'assoc' second argument is not a list"));
  while (list != NULL) {
    object *pair = first(list);
    if (eq(key,car(pair))) return pair;
    list = cdr(list);
  }
  return nil;
}

object *fn_member (object *args, object *env) {
  (void) env;
  object *item = first(args);
  object *list = second(args);
  if (!listp(list)) error(F("'member' second argument is not a list"));
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
  if (!listp(car(last))) error(F("'apply' last argument is not a list"));
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
    if (!listp(list)) error(F("'append' argument is not a list"));
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
  if (!listp(list1)) error(F("'mapc' second argument is not a list"));
  object *list2 = third(args);
  if (!listp(list2)) error(F("'mapc' third argument is not a list"));
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
  if (!listp(list1)) error(F("'mapcar' second argument is not a list"));
  object *list2 = third(args);
  if (!listp(list2)) error(F("'mapcar' third argument is not a list"));
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
    if (temp < 1) { if (-32768 - temp > result) error(F("'+' arithmetic overflow")); }
    else { if (32767 - temp < result) error(F("'+' arithmetic overflow")); }
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
    if (result == -32768) error(F("'-' arithmetic overflow"));
    #endif
    return number(-result);
  }
  while (args != NULL) {
    int temp = integer(car(args));
    #if defined(checkoverflow)
    if (temp < 1) { if (32767 + temp < result) error(F("'-' arithmetic overflow")); }
    else { if (-32768 + temp > result) error(F("'-' arithmetic overflow")); }
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
    if ((temp > 32767) || (temp < -32768)) error(F("'*' arithmetic overflow"));
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
    if (arg == 0) error(F("Division by zero"));
    #if defined(checkoverflow)
    if ((result == -32768) && (arg == -1)) error(F("'/' arithmetic overflow"));
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
  if (arg2 == 0) error(F("Division by zero"));
  int r = arg1 % arg2;
  if ((arg1<0) != (arg2<0)) r = r + arg2;
  return number(r);
}

object *fn_oneplus (object *args, object *env) {
  (void) env;
  int result = integer(first(args));
  #if defined(checkoverflow)
  if (result == 32767) error(F("'1+' arithmetic overflow"));
  #endif
  return number(result + 1);
}

object *fn_oneminus (object *args, object *env) {
  (void) env;
  int result = integer(first(args));
  #if defined(checkoverflow)
  if (result == -32768) error(F("'1-' arithmetic overflow"));
  #endif
  return number(result - 1);
}

object *fn_abs (object *args, object *env) {
  (void) env;
  int result = integer(first(args));
  #if defined(checkoverflow)
  if (result == -32768) error(F("'abs' arithmetic overflow"));
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

// System functions

object *fn_read (object *args, object *env) {
  (void) args;
  (void) env;
  return read();
}

object *fn_eval (object *args, object *env) {
  return eval(first(args), env);
}

object *fn_locals (object *args, object *env) {
  (void) args;
  return env;
}

object *fn_globals (object *args, object *env) {
  (void) args;
  (void) env;
  return GlobalEnv;
}

object *fn_makunbound (object *args, object *env) {
  (void) args;
  (void) env;
  object *key = first(args);
  object *list = GlobalEnv;
  object *prev = NULL;
  while (list != NULL) {
    object *pair = first(list);
    if (eq(key,car(pair))) {
      if (prev == NULL) GlobalEnv = cdr(list);
      else cdr(prev) = cdr(list);
      return key;
    }
    prev = list;
    list = cdr(list);
  }
  error2(key, F("not found"));
  return nil;
}

object *fn_break (object *args, object *env) {
  (void) args;
  Serial.println();
  Serial.println(F("Break!"));
  BreakLevel++;
  repl(env);
  BreakLevel--;
  return nil;
}

object *fn_print (object *args, object *env) {
  (void) env;
  Serial.println();
  object *obj = first(args);
  printobject(obj);
  Serial.print(' ');
  return obj;
}

object *fn_princ (object *args, object *env) {
  (void) env;
  object *obj = first(args);
  printobject(obj);
  return obj;
}

object *fn_gc (object *obj, object *env) {
  unsigned long start = micros();
  int initial = freespace;
  gc(obj, env);
  Serial.print(F("Space: "));
  Serial.print(freespace - initial);
  Serial.print(F(" bytes, Time: "));
  Serial.print(micros() - start);
  Serial.println(F(" uS"));
  return nil;
}

object *fn_saveimage (object *args, object *env) {
  object *var = eval(first(args), env);
  return number(saveimage(var));
}

object *fn_loadimage (object *args, object *env) {
  (void) args;
  (void) env;
  return number(loadimage());
}

// Arduino procedures

object *fn_pinmode (object *args, object *env) {
  (void) env;
  int pin = integer(first(args));
  object *mode = second(args);
  if (mode->type == NUMBER) pinMode(pin, mode->integer);
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
#if defined(__AVR_ATmega328P__)
  if (!(pin>=0 && pin<=5)) error(F("'analogread' invalid pin"));
#elif defined(__AVR_ATmega2560__)
  if (!(pin>=0 && pin<=15)) error(F("'analogread' invalid pin"));
#endif
  return number(analogRead(pin));
}
 
object *fn_analogwrite (object *args, object *env) {
  (void) env;
  int pin = integer(first(args));
#if defined(__AVR_ATmega328P__)
  if (!(pin>=3 && pin<=11 && pin!=4 && pin!=7 && pin!=8)) error(F("'analogwrite' invalid pin"));
#elif defined(__AVR_ATmega2560__)
  if (!((pin>=2 && pin<=13) || (pin>=44 && pin <=46))) error(F("'analogwrite' invalid pin"));
#endif
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
  (void) env;
  (void) args;
  unsigned long temp = millis();
  #if defined(checkoverflow)
  if (temp > 32767) error(F("'millis' arithmetic overflow"));
  #endif
  return number(temp);
}

object *fn_shiftout (object *args, object *env) {
  (void) env;
  int datapin = integer(first(args));
  int clockpin = integer(second(args));
  int order = (third(args) != nil);
  object *value = fourth(args);
  shiftOut(datapin, clockpin, order, integer(value));
  return value;
}

object *fn_shiftin (object *args, object *env) {
  (void) env;
  int datapin = integer(first(args));
  int clockpin = integer(second(args));
  int order = (third(args) != nil);
  int value = shiftIn(datapin, clockpin, order);
  return number(value);
}

const uint8_t scale[] PROGMEM = { 239,225,213,201,190,179,169,159,150,142,134,127};

object *fn_note (object *args, object *env) {
  (void) env;
  #if defined(__AVR_ATmega328P__)
  if (args != NULL) {
    int pin = integer(first(args));
    int note = integer(second(args));
    if (pin == 3) {
      DDRD = DDRD | 1<<DDD3; // PD3 (Arduino D3) as output
      TCCR2A = 0<<COM2A0 | 1<<COM2B0 | 2<<WGM20; // Toggle OC2B on match
    } else if (pin == 11) {
      DDRB = DDRB | 1<<DDB3; // PB3 (Arduino D11) as output
      TCCR2A = 1<<COM2A0 | 0<<COM2B0 | 2<<WGM20; // Toggle OC2A on match
    } else error(F("'note' pin not supported"));
    int prescaler = 0;
    if (cdr(cdr(args)) != NULL) prescaler = integer(third(args));
    prescaler = 9 - prescaler - note/12;
    if (prescaler<3 || prescaler>6) error(F("'note' octave out of range"));
    OCR2A = pgm_read_byte(&scale[note%12]);
    TCCR2B = 0<<WGM22 | prescaler<<CS20;
  } else TCCR2B = 0<<WGM22 | 0<<CS20;
  
  #elif defined(__AVR_ATmega2560__)
  if (args != NULL) {
    int pin = integer(first(args));
    int note = integer(second(args));
    if (pin == 9) {
      DDRH = DDRH | 1<<DDH6; // PH6 (Arduino D9) as output
      TCCR2A = 0<<COM2A0 | 1<<COM2B0 | 2<<WGM20; // Toggle OC2B on match
    } else if (pin == 10) {
      DDRB = DDRB | 1<<DDB4; // PB4 (Arduino D10) as output
      TCCR2A = 1<<COM2A0 | 0<<COM2B0 | 2<<WGM20; // Toggle OC2A on match
    } else error(F("'note' pin not supported"));
    int prescaler = 0;
    if (cdr(cdr(args)) != NULL) prescaler = integer(third(args));
    prescaler = 9 - prescaler - note/12;
    if (prescaler<3 || prescaler>6) error(F("'note' octave out of range"));
    OCR2A = pgm_read_byte(&scale[note%12]);
    TCCR2B = 0<<WGM22 | prescaler<<CS20;
  } else TCCR2B = 0<<WGM22 | 0<<CS20;
  
  #elif defined(__AVR_ATmega1284P__)
  if (args != NULL) {
    int pin = integer(first(args));
    int note = integer(second(args));
    if (pin == 14) {
      DDRD = DDRD | 1<<DDD6; // PD6 (Arduino D14) as output
      TCCR2A = 0<<COM2A0 | 1<<COM2B0 | 2<<WGM20; // Toggle OC2B on match
    } else if (pin == 15) {
      DDRD = DDRD | 1<<DDD7; // PD7 (Arduino D15) as output
      TCCR2A = 1<<COM2A0 | 0<<COM2B0 | 2<<WGM20; // Toggle OC2A on match
    } else error(F("'note' pin not supported"));
    int prescaler = 0;
    if (cdr(cdr(args)) != NULL) prescaler = integer(third(args));
    prescaler = 9 - prescaler - note/12;
    if (prescaler<3 || prescaler>6) error(F("'note' octave out of range"));
    OCR2A = pgm_read_byte(&scale[note%12]);
    TCCR2B = 0<<WGM22 | prescaler<<CS20;
  } else TCCR2B = 0<<WGM22 | 0<<CS20;
  #endif
  return nil;
}

// Insert your own function definitions here


// Built-in procedure names - stored in PROGMEM

const char string0[] PROGMEM = "symbols";
const char string1[] PROGMEM = "nil";
const char string2[] PROGMEM = "t";
const char string3[] PROGMEM = "lambda";
const char string4[] PROGMEM = "let";
const char string5[] PROGMEM = "let*";
const char string6[] PROGMEM = "closure";
const char string7[] PROGMEM = "special_forms";
const char string8[] PROGMEM = "quote";
const char string9[] PROGMEM = "defun";
const char string10[] PROGMEM = "defvar";
const char string11[] PROGMEM = "setq";
const char string12[] PROGMEM = "loop";
const char string13[] PROGMEM = "push";
const char string14[] PROGMEM = "pop";
const char string15[] PROGMEM = "incf";
const char string16[] PROGMEM = "decf";
const char string17[] PROGMEM = "dolist";
const char string18[] PROGMEM = "dotimes";
const char string19[] PROGMEM = "for-millis";
const char string20[] PROGMEM = "tail_forms";
const char string21[] PROGMEM = "progn";
const char string22[] PROGMEM = "return";
const char string23[] PROGMEM = "if";
const char string24[] PROGMEM = "cond";
const char string25[] PROGMEM = "when";
const char string26[] PROGMEM = "unless";
const char string27[] PROGMEM = "and";
const char string28[] PROGMEM = "or";
const char string29[] PROGMEM = "functions";
const char string30[] PROGMEM = "not";
const char string31[] PROGMEM = "null";
const char string32[] PROGMEM = "cons";
const char string33[] PROGMEM = "atom";
const char string34[] PROGMEM = "listp";
const char string35[] PROGMEM = "consp";
const char string36[] PROGMEM = "numberp";
const char string37[] PROGMEM = "eq";
const char string38[] PROGMEM = "car";
const char string39[] PROGMEM = "first";
const char string40[] PROGMEM = "cdr";
const char string41[] PROGMEM = "rest";
const char string42[] PROGMEM = "caar";
const char string43[] PROGMEM = "cadr";
const char string44[] PROGMEM = "second";
const char string45[] PROGMEM = "cdar";
const char string46[] PROGMEM = "cddr";
const char string47[] PROGMEM = "caaar";
const char string48[] PROGMEM = "caadr";
const char string49[] PROGMEM = "cadar";
const char string50[] PROGMEM = "caddr";
const char string51[] PROGMEM = "third";
const char string52[] PROGMEM = "cdaar";
const char string53[] PROGMEM = "cdadr";
const char string54[] PROGMEM = "cddar";
const char string55[] PROGMEM = "cdddr";
const char string56[] PROGMEM = "length";
const char string57[] PROGMEM = "list";
const char string58[] PROGMEM = "reverse";
const char string59[] PROGMEM = "nth";
const char string60[] PROGMEM = "assoc";
const char string61[] PROGMEM = "member";
const char string62[] PROGMEM = "apply";
const char string63[] PROGMEM = "funcall";
const char string64[] PROGMEM = "append";
const char string65[] PROGMEM = "mapc";
const char string66[] PROGMEM = "mapcar";
const char string67[] PROGMEM = "+";
const char string68[] PROGMEM = "-";
const char string69[] PROGMEM = "*";
const char string70[] PROGMEM = "/";
const char string71[] PROGMEM = "mod";
const char string72[] PROGMEM = "1+";
const char string73[] PROGMEM = "1-";
const char string74[] PROGMEM = "abs";
const char string75[] PROGMEM = "random";
const char string76[] PROGMEM = "max";
const char string77[] PROGMEM = "min";
const char string78[] PROGMEM = "=";
const char string79[] PROGMEM = "<";
const char string80[] PROGMEM = "<=";
const char string81[] PROGMEM = ">";
const char string82[] PROGMEM = ">=";
const char string83[] PROGMEM = "/=";
const char string84[] PROGMEM = "plusp";
const char string85[] PROGMEM = "minusp";
const char string86[] PROGMEM = "zerop";
const char string87[] PROGMEM = "oddp";
const char string88[] PROGMEM = "evenp";
const char string89[] PROGMEM = "read";
const char string90[] PROGMEM = "eval";
const char string91[] PROGMEM = "locals";
const char string92[] PROGMEM = "globals";
const char string93[] PROGMEM = "makunbound";
const char string94[] PROGMEM = "break";
const char string95[] PROGMEM = "print";
const char string96[] PROGMEM = "princ";
const char string97[] PROGMEM = "gc";
const char string98[] PROGMEM = "save-image";
const char string99[] PROGMEM = "load-image";
const char string100[] PROGMEM = "pinmode";
const char string101[] PROGMEM = "digitalread";
const char string102[] PROGMEM = "digitalwrite";
const char string103[] PROGMEM = "analogread";
const char string104[] PROGMEM = "analogwrite";
const char string105[] PROGMEM = "delay";
const char string106[] PROGMEM = "millis";
const char string107[] PROGMEM = "shiftout";
const char string108[] PROGMEM = "shiftin";
const char string109[] PROGMEM = "note";

const tbl_entry_t lookup_table[] PROGMEM = {
  { string0, NULL, NIL, NIL },
  { string1, NULL, 0, 0 },
  { string2, NULL, 1, 0 },
  { string3, NULL, 0, 127 },
  { string4, NULL, 0, 127 },
  { string5, NULL, 0, 127 },
  { string6, NULL, 0, 127 },
  { string7, NULL, NIL, NIL },
  { string8, sp_quote, 1, 1 },
  { string9, sp_defun, 0, 127 },
  { string10, sp_defvar, 0, 127 },
  { string11, sp_setq, 2, 2 },
  { string12, sp_loop, 0, 127 },
  { string13, sp_push, 2, 2 },
  { string14, sp_pop, 1, 1 },
  { string15, sp_incf, 1, 2 },
  { string16, sp_decf, 1, 2 },
  { string17, sp_dolist, 1, 127 },
  { string18, sp_dotimes, 1, 127 },
  { string19, sp_formillis, 1, 127 },
  { string20, NULL, NIL, NIL },
  { string21, tf_progn, 0, 127 },
  { string22, tf_return, 0, 127 },
  { string23, tf_if, 2, 3 },
  { string24, tf_cond, 0, 127 },
  { string25, tf_when, 1, 127 },
  { string26, tf_unless, 1, 127 },
  { string27, tf_and, 0, 127 },
  { string28, tf_or, 0, 127 },
  { string29, NULL, NIL, NIL },
  { string30, fn_not, 1, 1 },
  { string31, fn_not, 1, 1 },
  { string32, fn_cons, 2, 2 },
  { string33, fn_atom, 1, 1 },
  { string34, fn_listp, 1, 1 },
  { string35, fn_consp, 1, 1 },
  { string36, fn_numberp, 1, 1 },
  { string37, fn_eq, 2, 2 },
  { string38, fn_car, 1, 1 },
  { string39, fn_car, 1, 1 },
  { string40, fn_cdr, 1, 1 },
  { string41, fn_cdr, 1, 1 },
  { string42, fn_caar, 1, 1 },
  { string43, fn_cadr, 1, 1 },
  { string44, fn_cadr, 1, 1 },
  { string45, fn_cdar, 1, 1 },
  { string46, fn_cddr, 1, 1 },
  { string47, fn_caaar, 1, 1 },
  { string48, fn_caadr, 1, 1 },
  { string49, fn_cadar, 1, 1 },
  { string50, fn_caddr, 1, 1 },
  { string51, fn_caddr, 1, 1 },
  { string52, fn_cdaar, 1, 1 },
  { string53, fn_cdadr, 1, 1 },
  { string54, fn_cddar, 1, 1 },
  { string55, fn_cdddr, 1, 1 },
  { string56, fn_length, 1, 1 },
  { string57, fn_list, 0, 127 },
  { string58, fn_reverse, 1, 1 },
  { string59, fn_nth, 2, 2 },
  { string60, fn_assoc, 2, 2 },
  { string61, fn_member, 2, 2 },
  { string62, fn_apply, 2, 127 },
  { string63, fn_funcall, 1, 127 },
  { string64, fn_append, 0, 127 },
  { string65, fn_mapc, 2, 3 },
  { string66, fn_mapcar, 2, 3 },
  { string67, fn_add, 0, 127 },
  { string68, fn_subtract, 1, 127 },
  { string69, fn_multiply, 0, 127 },
  { string70, fn_divide, 2, 127 },
  { string71, fn_mod, 2, 2 },
  { string72, fn_oneplus, 1, 1 },
  { string73, fn_oneminus, 1, 1 },
  { string74, fn_abs, 1, 1 },
  { string75, fn_random, 1, 1 },
  { string76, fn_max, 1, 127 },
  { string77, fn_min, 1, 127 },
  { string78, fn_numeq, 1, 127 },
  { string79, fn_less, 1, 127 },
  { string80, fn_lesseq, 1, 127 },
  { string81, fn_greater, 1, 127 },
  { string82, fn_greatereq, 1, 127 },
  { string83, fn_noteq, 1, 127 },
  { string84, fn_plusp, 1, 1 },
  { string85, fn_minusp, 1, 1 },
  { string86, fn_zerop, 1, 1 },
  { string87, fn_oddp, 1, 1 },
  { string88, fn_evenp, 1, 1 },
  { string89, fn_read, 0, 0 },
  { string90, fn_eval, 1, 1 },
  { string91, fn_locals, 0, 0 },
  { string92, fn_globals, 0, 0 },
  { string93, fn_makunbound, 1, 1 },
  { string94, fn_break, 0, 0 },
  { string95, fn_print, 1, 1 },
  { string96, fn_princ, 1, 1 },
  { string97, fn_gc, 0, 0 },
  { string98, fn_saveimage, 0, 1 },
  { string99, fn_loadimage, 0, 0 },
  { string100, fn_pinmode, 2, 2 },
  { string101, fn_digitalread, 1, 1 },
  { string102, fn_digitalwrite, 2, 2 },
  { string103, fn_analogread, 1, 1 },
  { string104, fn_analogwrite, 2, 2 },
  { string105, fn_delay, 1, 1 },
  { string106, fn_millis, 0, 0 },
  { string107, fn_shiftout, 4, 4 },
  { string108, fn_shiftin, 3, 3 },
  { string109, fn_note, 0, 3 },
};

// Table lookup functions

int builtin(char* n){
  int entry = 0;
  while (entry < ENDFUNCTIONS) {
    if(strcmp_P(n, (char*)pgm_read_word(&lookup_table[entry].string)) == 0)
      return entry;
    entry++;
  }
  return ENDFUNCTIONS;
}

int lookupfn(unsigned int name) {
  return pgm_read_word(&lookup_table[name].fptr);
}

int lookupmin(unsigned int name) {
  return pgm_read_word(&lookup_table[name].min);
}

int lookupmax(unsigned int name) {
  return pgm_read_word(&lookup_table[name].max);
}

char *lookupstring (unsigned int name) {
  strcpy_P(buffer, (char *)(pgm_read_word(&lookup_table[name].string)));
  return buffer;
}

// Main evaluator

// unsigned int Canary = 0xA5A5;

object *eval (object *form, object *env) {
  int TC=0;
  EVAL:
  // Enough space?
  if (freespace < 10) gc(form, env);
  if (_end != 0xA5) error(F("Error: Stack overflow"));
  // Break
  if (Serial.read() == '~') error(F("Break!"));
  
  if (form == NULL) return nil;

  if (form->type == NUMBER) return form;

  if (form->type == SYMBOL) {
    unsigned int name = form->name;
    if (name == NIL) return nil;
    object *pair = value(name, env);
    if (pair != NULL) return cdr(pair);
    pair = value(name, GlobalEnv);
    if (pair != NULL) return cdr(pair);
    else if (name <= ENDFUNCTIONS) return form;
    error2(form, F("undefined"));
  }
  
  // It's a list
  object *function = car(form);
  object *args = cdr(form);

  // List starts with a symbol?
  if (function->type == SYMBOL) {
    unsigned int name = function->name;

    if ((name == LET) || (name == LETSTAR)) {
      object *assigns = first(args);
      object *forms = cdr(args);
      object *newenv = env;
      while (assigns != NULL) {
        object *assign = car(assigns);
        if (consp(assign)) push(cons(first(assign),eval(second(assign),env)), newenv);
        else push(cons(assign,nil), newenv);
        if (name == LETSTAR) env = newenv;
        assigns = cdr(assigns);
      }
      env = newenv;
      form = tf_progn(forms,env);
      TC = 1;
      goto EVAL;
    }

    if (name == LAMBDA) {
      if (env == NULL) return form;
      object *envcopy = NULL;
      while (env != NULL) {
        object *pair = first(env);
        object *val = cdr(pair);
        if (val->type == NUMBER) val = number(val->integer);
        push(cons(car(pair), val), envcopy);
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
 
  if (function->type == SYMBOL) {
    unsigned int name = function->name;
    if (name >= ENDFUNCTIONS) error2(fname, F("is not a function"));
    if (nargs<lookupmin(name)) error2(fname, F("has too few arguments"));
    if (nargs>lookupmax(name)) error2(fname, F("has too many arguments"));
    object *result = ((fn_ptr_type)lookupfn(name))(args, env);
    pop(GCStack);
    return result;
  }
      
  if (listp(function) && issymbol(car(function), LAMBDA)) {
    form = closure(TCstart, fname, NULL, cdr(function), args, &env);
    pop(GCStack);
    TC = 1;
    goto EVAL;
  }

  if (listp(function) && issymbol(car(function), CLOSURE)) {
    function = cdr(function);
    form = closure(TCstart, fname, car(function), cdr(function), args, &env);
    pop(GCStack);
    TC = 1;
    goto EVAL;
  }    
  
  error2(fname, F("is an illegal function")); return nil;
}

// Input/Output

void printobject(object *form){
  #if defined(debug2)
  Serial.print('[');Serial.print((int)form);Serial.print(']');
  #endif
  if (form == NULL) Serial.print(F("nil"));
  else if (listp(form) && issymbol(car(form), CLOSURE)) Serial.print(F("<closure>"));
  else if (listp(form)) {
    Serial.print('(');
    printobject(car(form));
    form = cdr(form);
    while (form != NULL && listp(form)) {
      Serial.print(' ');
      printobject(car(form));
      form = cdr(form);
    }
    if (form != NULL) {
      Serial.print(F(" . "));
      printobject(form);
    }
    Serial.print(')');
  } else if (form->type == NUMBER){
    Serial.print(integer(form));
  } else if (form->type == SYMBOL){
    Serial.print(name(form));
  } else
    error(F("Error in print."));
}

int Getc () {
  if (LastChar) { 
    int temp = LastChar;
    LastChar = 0;
    return temp;
  }
  while (!Serial.available());
  int temp = Serial.read();
  Serial.print((char)temp);
  // if (temp == 13) Serial.println();
  return temp;
}

object *nextitem() {
  int ch = Getc();
  while(isspace(ch)) ch = Getc();

  if (ch == ';') {
    while(ch != '(') ch = Getc();
    ch = '(';
  }
  if (ch == '\n') ch = Getc();
  if (ch == EOF) exit(0);

  if (ch == ')') return (object *)KET;
  if (ch == '(') return (object *)BRA;
  if (ch == '\'') return (object *)QUO;
  if (ch == '.') return (object *)DOT;

  // Parse variable or number
  int index = 0, base = 10, sign = 1;
  unsigned int result = 0;
  if (ch == '+') {
    buffer[index++] = ch;
    ch = Getc();
  } else if (ch == '-') {
    sign = -1;
    buffer[index++] = ch;
    ch = Getc();
  } else if (ch == '#') {
    ch = Getc() | 0x20;
    if (ch == 'b') base = 2;
    else if (ch == 'o') base = 8;
    else if (ch == 'x') base = 16;
    else error(F("Illegal character after #"));
    ch = Getc();
  }
  int isnumber = (digitvalue(ch)<base);
  buffer[2] = '\0'; // In case variable is one letter

  while(!isspace(ch) && ch != ')' && index < buflen){
    buffer[index++] = ch;
    int temp = digitvalue(ch);
    result = result * base + temp;
    isnumber = isnumber && (digitvalue(ch)<base);
    ch = Getc();
  }

  buffer[index] = '\0';
  if (ch == ')') LastChar = ')';

  if (isnumber) {
    if (base == 10 && result > ((unsigned int)32767+(1-sign)/2)) {
      Serial.println();
      error(F("Number out of range"));
    }
    return number(result*sign);
  }
  
  int x = builtin(buffer);
  if (x == NIL) return nil;
  if (x < ENDFUNCTIONS) return symbol(x);
  else return symbol(pack40(buffer));
}

object *readrest() {
  object *item = nextitem();

  if(item == (object *)KET) return NULL;
  
  if(item == (object *)DOT) {
    object *arg1 = read();
    if (readrest() != NULL) error(F("Malformed list"));
    return arg1;
  }

  if(item == (object *)QUO) {
    object *arg1 = read();
    return cons(cons(symbol(QUOTE), cons(arg1, NULL)), readrest());
  }
   
  if(item == (object *)BRA) item = readrest(); 
  return cons(item, readrest());
}

object *read() {
  object *item = nextitem();
  if (item == (object *)BRA) return readrest();
  if (item == (object *)DOT) return read();
  if (item == (object *)QUO) return cons(symbol(QUOTE), cons(read(), NULL)); 
  return item;
}

void initenv() {
  GlobalEnv = NULL;
  tee = symbol(TEE);
}

// Setup

void setup() {
  Serial.begin(9600);
  while (!Serial);  // wait for Serial to initialize
  initworkspace();
  initenv();
  _end = 0xA5;
  Serial.println(F("uLisp 1.1"));
}

// Read/Evaluate/Print loop

void repl(object *env) {
  for (;;) {
    randomSeed(micros());
    gc(NULL, env);
    Serial.print(freespace);
    if (BreakLevel) {
      Serial.print(F(" : "));
      Serial.print(BreakLevel);
    }
    Serial.print(F("> "));
    object *line = read();
    if (line == nil) { Serial.println(); return; }
    Serial.println();
    push(line, GCStack);
    printobject(eval(line,env));
    pop(GCStack);
    Serial.println();
    Serial.println();
  }
}

void loop() {
  if (!setjmp(exception)) {
    #if defined(resetautorun)
    object *autorun = (object *)eeprom_read_word(&image.eval);
    if (autorun != NULL && (unsigned int)autorun != 0xFFFF) {
      loadimage();
      apply(autorun, NULL, NULL);
    }
    #endif
  }
  repl(NULL);
}