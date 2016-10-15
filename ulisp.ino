/* uLisp Version 1.0 - www.ulisp.com
    
   Copyright (c) 2016 David Johnson-Davies
   
   Licensed under the MIT license: https://opensource.org/licenses/MIT
*/

#include <setjmp.h>

// Compile options

#define checkoverflow

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
const int workspacesize = 315;
#elif defined(__AVR_ATmega32U4__)
const int workspacesize = 421;
#elif defined(__AVR_ATmega2560__)
const int workspacesize = 1461;
#elif defined(__AVR_ATmega1284P__)
const int workspacesize = 3000;
#else
const int workspacesize = 315;
#endif

const int buflen = 13;  // Length of longest symbol + 1
enum type {NONE, SYMBOL, NUMBER};

enum function { SYMBOLS, BRA, KET, QUO, DOT, NIL, TEE, LAMBDA, LET, LETSTAR, CLOSURE, SPECIAL_FORMS, QUOTE,
DEFUN, DEFVAR, SETQ, LOOP, PUSH, POP, INCF, DECF, DOLIST, DOTIMES, FORMILLIS, TAIL_FORMS, PROGN, RETURN, IF,
COND, WHEN, UNLESS, AND, OR, FUNCTIONS, NOT, NULLFN, CONS, ATOM, LISTP, CONSP, NUMBERP, EQ, CAR, FIRST, CDR,
REST, CAAR, CADR, SECOND, CDAR, CDDR, CAAAR, CAADR, CADAR, CADDR, THIRD, CDAAR, CDADR, CDDAR, CDDDR, LENGTH,
LIST, REVERSE, NTH, ASSOC, MEMBER, APPLY, FUNCALL, APPEND, MAPC, MAPCAR, ADD, SUBTRACT, MULTIPLY, DIVIDE, MOD,
ONEPLUS, ONEMINUS, ABS, RANDOM, MAX, MIN, NUMEQ, LESS, LESSEQ, GREATER, GREATEREQ, NOTEQ, PLUSP, MINUSP, ZEROP,
ODDP, EVENP, READ, EVAL, LOCALS, GLOBALS, MAKUNBOUND, BREAK, PRINT, PRINC, GC, PINMODE, DIGITALREAD, DIGITALWRITE,
ANALOGREAD, ANALOGWRITE, DELAY, MILLIS, SHIFTOUT, SHIFTIN, NOTE, ENDFUNCTIONS };

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
int ReturnFlag = 0;
object *freelist;

object *GlobalEnv;
object *GCStack = NULL;
char buffer[buflen+1];
int BreakLevel = 0;

// Forward references
object *tee, *bra, *ket, *quo, *dot;

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
  for (int i=0; i<workspacesize; i++) {
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
  if (obj == NULL) return;
  
  object* arg = car(obj);
  if (marked(obj)) return;

  int type = obj->type;
  mark(obj);
  
  if (type != SYMBOL && type != NUMBER) { // cons
    markobject(arg);
    markobject(cdr(obj));
  }
}

void sweep () {
  freelist = NULL;
  freespace = 0;
  for (int i=0; i<workspacesize; i++) {
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
  markobject(bra); markobject(ket); markobject(quo);
  markobject(tee); markobject(dot);
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
  if (ch >= 'A' && ch <= 'Z') return ch-'A'+1;
  if (ch >= 'a' && ch <= 'z') return ch-'a'+1;
  if (ch >= '0' && ch <= '9') return ch-'0'+30;
  if (ch == '-') return 27;
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

object *closure (int tail, object *fname, object *function, object *args, object **env) {
  object *local = first(function);
  object *params = second(function);
  function = cdr(cdr(function));
  // Push locals
   while (local != NULL) {
    push(first(local), *env);
    local = cdr(local);
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
  } else if (listp(function) && issymbol(car(function), CLOSURE)) {
    object *result = closure(0, NULL, cdr(function), args, env);
    return eval(result,*env);
  } else error2(function, F("illegal function"));
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
  object *val = cons(symbol(CLOSURE), cons(NULL,cdr(args)));
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
  return nil;
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
  #endif
  return nil;
}

// Insert your own function definitions here


// Built-in procedure names - stored in PROGMEM

const char string0[] PROGMEM = "symbols";
const char string1[] PROGMEM = "(";
const char string2[] PROGMEM = ")";
const char string3[] PROGMEM = "'";
const char string4[] PROGMEM = ".";
const char string5[] PROGMEM = "nil";
const char string6[] PROGMEM = "t";
const char string7[] PROGMEM = "lambda";
const char string8[] PROGMEM = "let";
const char string9[] PROGMEM = "let*";
const char string10[] PROGMEM = "closure";
const char string11[] PROGMEM = "special_forms";
const char string12[] PROGMEM = "quote";
const char string13[] PROGMEM = "defun";
const char string14[] PROGMEM = "defvar";
const char string15[] PROGMEM = "setq";
const char string16[] PROGMEM = "loop";
const char string17[] PROGMEM = "push";
const char string18[] PROGMEM = "pop";
const char string19[] PROGMEM = "incf";
const char string20[] PROGMEM = "decf";
const char string21[] PROGMEM = "dolist";
const char string22[] PROGMEM = "dotimes";
const char string23[] PROGMEM = "for-millis";
const char string24[] PROGMEM = "tail_forms";
const char string25[] PROGMEM = "progn";
const char string26[] PROGMEM = "return";
const char string27[] PROGMEM = "if";
const char string28[] PROGMEM = "cond";
const char string29[] PROGMEM = "when";
const char string30[] PROGMEM = "unless";
const char string31[] PROGMEM = "and";
const char string32[] PROGMEM = "or";
const char string33[] PROGMEM = "functions";
const char string34[] PROGMEM = "not";
const char string35[] PROGMEM = "null";
const char string36[] PROGMEM = "cons";
const char string37[] PROGMEM = "atom";
const char string38[] PROGMEM = "listp";
const char string39[] PROGMEM = "consp";
const char string40[] PROGMEM = "numberp";
const char string41[] PROGMEM = "eq";
const char string42[] PROGMEM = "car";
const char string43[] PROGMEM = "first";
const char string44[] PROGMEM = "cdr";
const char string45[] PROGMEM = "rest";
const char string46[] PROGMEM = "caar";
const char string47[] PROGMEM = "cadr";
const char string48[] PROGMEM = "second";
const char string49[] PROGMEM = "cdar";
const char string50[] PROGMEM = "cddr";
const char string51[] PROGMEM = "caaar";
const char string52[] PROGMEM = "caadr";
const char string53[] PROGMEM = "cadar";
const char string54[] PROGMEM = "caddr";
const char string55[] PROGMEM = "third";
const char string56[] PROGMEM = "cdaar";
const char string57[] PROGMEM = "cdadr";
const char string58[] PROGMEM = "cddar";
const char string59[] PROGMEM = "cdddr";
const char string60[] PROGMEM = "length";
const char string61[] PROGMEM = "list";
const char string62[] PROGMEM = "reverse";
const char string63[] PROGMEM = "nth";
const char string64[] PROGMEM = "assoc";
const char string65[] PROGMEM = "member";
const char string66[] PROGMEM = "apply";
const char string67[] PROGMEM = "funcall";
const char string68[] PROGMEM = "append";
const char string69[] PROGMEM = "mapc";
const char string70[] PROGMEM = "mapcar";
const char string71[] PROGMEM = "+";
const char string72[] PROGMEM = "-";
const char string73[] PROGMEM = "*";
const char string74[] PROGMEM = "/";
const char string75[] PROGMEM = "mod";
const char string76[] PROGMEM = "1+";
const char string77[] PROGMEM = "1-";
const char string78[] PROGMEM = "abs";
const char string79[] PROGMEM = "random";
const char string80[] PROGMEM = "max";
const char string81[] PROGMEM = "min";
const char string82[] PROGMEM = "=";
const char string83[] PROGMEM = "<";
const char string84[] PROGMEM = "<=";
const char string85[] PROGMEM = ">";
const char string86[] PROGMEM = ">=";
const char string87[] PROGMEM = "/=";
const char string88[] PROGMEM = "plusp";
const char string89[] PROGMEM = "minusp";
const char string90[] PROGMEM = "zerop";
const char string91[] PROGMEM = "oddp";
const char string92[] PROGMEM = "evenp";
const char string93[] PROGMEM = "read";
const char string94[] PROGMEM = "eval";
const char string95[] PROGMEM = "locals";
const char string96[] PROGMEM = "globals";
const char string97[] PROGMEM = "makunbound";
const char string98[] PROGMEM = "break";
const char string99[] PROGMEM = "print";
const char string100[] PROGMEM = "princ";
const char string101[] PROGMEM = "gc";
const char string102[] PROGMEM = "pinmode";
const char string103[] PROGMEM = "digitalread";
const char string104[] PROGMEM = "digitalwrite";
const char string105[] PROGMEM = "analogread";
const char string106[] PROGMEM = "analogwrite";
const char string107[] PROGMEM = "delay";
const char string108[] PROGMEM = "millis";
const char string109[] PROGMEM = "shiftout";
const char string110[] PROGMEM = "shiftin";
const char string111[] PROGMEM = "note";

const tbl_entry_t lookup_table[] PROGMEM = {
  { string0, NULL, NIL, NIL },
  { string1, NULL, 0, 0 },
  { string2, NULL, 0, 0 },
  { string3, NULL, 0, 0 },
  { string4, NULL, 0, 0 },
  { string5, NULL, 0, 0 },
  { string6, NULL, 0, 0 },
  { string7, NULL, 0, 127 },
  { string8, NULL, 0, 127 },
  { string9, NULL, 0, 127 },
  { string10, NULL, 0, 127 },
  { string11, NULL, NIL, NIL },
  { string12, sp_quote, 1, 1 },
  { string13, sp_defun, 0, 127 },
  { string14, sp_defvar, 0, 127 },
  { string15, sp_setq, 2, 2 },
  { string16, sp_loop, 0, 127 },
  { string17, sp_push, 2, 2 },
  { string18, sp_pop, 1, 1 },
  { string19, sp_incf, 1, 2 },
  { string20, sp_decf, 1, 2 },
  { string21, sp_dolist, 1, 127 },
  { string22, sp_dotimes, 1, 127 },
  { string23, sp_formillis, 1, 127 },
  { string24, NULL, NIL, NIL },
  { string25, tf_progn, 0, 127 },
  { string26, tf_return, 0, 127 },
  { string27, tf_if, 2, 3 },
  { string28, tf_cond, 0, 127 },
  { string29, tf_when, 1, 127 },
  { string30, tf_unless, 1, 127 },
  { string31, tf_and, 0, 127 },
  { string32, tf_or, 0, 127 },
  { string33, NULL, NIL, NIL },
  { string34, fn_not, 1, 1 },
  { string35, fn_not, 1, 1 },
  { string36, fn_cons, 2, 2 },
  { string37, fn_atom, 1, 1 },
  { string38, fn_listp, 1, 1 },
  { string39, fn_consp, 1, 1 },
  { string40, fn_numberp, 1, 1 },
  { string41, fn_eq, 2, 2 },
  { string42, fn_car, 1, 1 },
  { string43, fn_car, 1, 1 },
  { string44, fn_cdr, 1, 1 },
  { string45, fn_cdr, 1, 1 },
  { string46, fn_caar, 1, 1 },
  { string47, fn_cadr, 1, 1 },
  { string48, fn_cadr, 1, 1 },
  { string49, fn_cdar, 1, 1 },
  { string50, fn_cddr, 1, 1 },
  { string51, fn_caaar, 1, 1 },
  { string52, fn_caadr, 1, 1 },
  { string53, fn_cadar, 1, 1 },
  { string54, fn_caddr, 1, 1 },
  { string55, fn_caddr, 1, 1 },
  { string56, fn_cdaar, 1, 1 },
  { string57, fn_cdadr, 1, 1 },
  { string58, fn_cddar, 1, 1 },
  { string59, fn_cdddr, 1, 1 },
  { string60, fn_length, 1, 1 },
  { string61, fn_list, 0, 127 },
  { string62, fn_reverse, 1, 1 },
  { string63, fn_nth, 2, 2 },
  { string64, fn_assoc, 2, 2 },
  { string65, fn_member, 2, 2 },
  { string66, fn_apply, 2, 127 },
  { string67, fn_funcall, 1, 127 },
  { string68, fn_append, 0, 127 },
  { string69, fn_mapc, 2, 3 },
  { string70, fn_mapcar, 2, 3 },
  { string71, fn_add, 0, 127 },
  { string72, fn_subtract, 1, 127 },
  { string73, fn_multiply, 0, 127 },
  { string74, fn_divide, 2, 127 },
  { string75, fn_mod, 2, 2 },
  { string76, fn_oneplus, 1, 1 },
  { string77, fn_oneminus, 1, 1 },
  { string78, fn_abs, 1, 1 },
  { string79, fn_random, 1, 1 },
  { string80, fn_max, 1, 127 },
  { string81, fn_min, 1, 127 },
  { string82, fn_numeq, 1, 127 },
  { string83, fn_less, 1, 127 },
  { string84, fn_lesseq, 1, 127 },
  { string85, fn_greater, 1, 127 },
  { string86, fn_greatereq, 1, 127 },
  { string87, fn_noteq, 1, 127 },
  { string88, fn_plusp, 1, 1 },
  { string89, fn_minusp, 1, 1 },
  { string90, fn_zerop, 1, 1 },
  { string91, fn_oddp, 1, 1 },
  { string92, fn_evenp, 1, 1 },
  { string93, fn_read, 0, 0 },
  { string94, fn_eval, 1, 1 },
  { string95, fn_locals, 0, 0 },
  { string96, fn_globals, 0, 0 },
  { string97, fn_makunbound, 1, 1 },
  { string98, fn_break, 0, 0 },
  { string99, fn_print, 1, 1 },
  { string100, fn_princ, 1, 1 },
  { string101, fn_gc, 0, 0 },
  { string102, fn_pinmode, 2, 2 },
  { string103, fn_digitalread, 1, 1 },
  { string104, fn_digitalwrite, 2, 2 },
  { string105, fn_analogread, 1, 1 },
  { string106, fn_analogwrite, 2, 2 },
  { string107, fn_delay, 1, 1 },
  { string108, fn_millis, 0, 0 },
  { string109, fn_shiftout, 4, 4 },
  { string110, fn_shiftin, 3, 3 },
  { string111, fn_note, 0, 3 },
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

unsigned int Canary = 0xA5A5;

object *eval (object *form, object *env) {
  int TC=0;
  EVAL:
  // Enough space?
  if (freespace < 10) gc(form, env);
  if (Canary != 0xA5A5) error(F("Error: Stack overflow"));
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
      
  if (listp(function) && issymbol(car(function), CLOSURE)) {
    form = closure(TCstart, fname, cdr(function), args, &env);
    pop(GCStack);
    TC = 1;
    goto EVAL;
  }    

  error2(fname, F("is an illegal function")); return nil;
}

// Input/Output

void printobject(object *form){
  if (form == NULL) Serial.print(F("nil"));
  else if (listp(form) && issymbol(car(form), CLOSURE) && car(cdr(form)) != NULL) {
    Serial.print(F("<closure>"));
  } else if (listp(form)) {
    Serial.print('(');
    if (issymbol(car(form), CLOSURE)) {
      Serial.print(F("lambda"));
      form = cdr(form);
    } else printobject(car(form));
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
    #if defined(debug2)
    Serial.print('[');Serial.print((int)form);Serial.print(']');
    #endif
  } else
    error(F("Error in print."));
}

int LastChar = 0;

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

  if (ch == ')') return ket;
  if (ch == '(') return bra;
  if (ch == '\'') return quo;
  if (ch == '.') return dot;

  int index = 0;
  if (ch == '+' || ch == '-') {
    buffer[index++] = ch;
    ch = Getc();
  }
  int isnumber = isdigit(ch);
  buffer[2] = '\0'; // In case variable is one letter

  while(!isspace(ch) && ch != ')' && index < buflen){
    buffer[index++] = ch;
    isnumber = isnumber && isdigit(ch);
    ch = Getc();
  }

  buffer[index] = '\0';
  if (ch == ')') LastChar = ')';

  if (isnumber) return number(atoi(buffer));
  
  int x = builtin(buffer);
  if (x == NIL) return nil;
  if (x < ENDFUNCTIONS) return symbol(x);
  else return symbol(pack40(buffer));
}

object *readrest() {
  object *item = nextitem();

  if(item == ket) return NULL;
  
  if(item == dot) {
    object *arg1 = read();
    if (readrest() != NULL) error(F("Malformed list"));
    return arg1;
  }

  if(item == quo) {
    object *arg1 = read();
    return cons(cons(symbol(QUOTE), cons(arg1, NULL)), readrest());
  }
   
  if(item == bra) item = readrest(); 
  return cons(item, readrest());
}

object *read() {
  object *item = nextitem();
  if (item == bra) return readrest();
  if (item == dot) return read();
  if (item == quo) return cons(symbol(QUOTE), cons(read(), NULL)); 
  return item;
}

void initenv() {
  GlobalEnv = NULL;
  bra = symbol(BRA); ket = symbol(KET); 
  tee = symbol(TEE); quo = symbol(QUO);
  dot = symbol(DOT);
}

// Setup

void setup() {
  Serial.begin(9600);
  while (!Serial);  // wait for Serial to initialize
  initworkspace();
  initenv();
  Serial.println(F("uLisp 1.0"));
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
    if (line == nil) return;
    Serial.println();
    push(line, GCStack);
    printobject(eval(line,env));
    pop(GCStack);
    Serial.println();
    Serial.println();
  }
}

void loop() {
  setjmp(exception);
  repl(NULL);
}
