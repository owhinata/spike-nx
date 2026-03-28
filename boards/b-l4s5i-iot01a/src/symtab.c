/* ../boards/b-l4s5i-iot01a/src/symtab.c: Auto-generated symbol table.  Do not edit */

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/symtab.h>

#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <ssp/ssp.h>
#include <debug.h>
#include <unistd.h>
#include <aio.h>
#include <dirent.h>
#include <stdio.h>
#include <fixedmath.h>
#include <libgen.h>
#include <libintl.h>
#include <wchar.h>
#include <termios.h>
#include <time.h>
#include <nuttx/crc32.h>
#include <dlfcn.h>
#include <nuttx/queue.h>
#include <netinet/ether.h>
#include <strings.h>
#include <fnmatch.h>
#include <netdb.h>
#include <sys/statvfs.h>
#include <sys/resource.h>
#include <pwd.h>
#include <sys/random.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <iconv.h>
#include <inttypes.h>
#include <ctype.h>
#include <wctype.h>
#include <nuttx/tls.h>
#include <malloc.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/uio.h>
#include <pthread.h>
#include <signal.h>
#include <sched.h>
#include <semaphore.h>
#include <locale.h>
#include <syslog.h>
#include <sys/socket.h>
#include <sys/utsname.h>

const struct symtab_s g_symtab[] =
{
  { "__assert", (FAR const void *)__assert },
  { "__cxa_atexit", (FAR const void *)__cxa_atexit },
#if defined(CONFIG_BUILD_FLAT)
  { "__errno", (FAR const void *)&__errno },
#endif
#if defined(CONFIG_STACK_CANARIES)
  { "__stack_chk_fail", (FAR const void *)__stack_chk_fail },
#endif
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG_ERROR)
  { "_alert", (FAR const void *)_alert },
#endif
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG_ERROR)
  { "_err", (FAR const void *)_err },
#endif
  { "_exit", (FAR const void *)_exit },
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG_INFO)
  { "_info", (FAR const void *)_info },
#endif
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG_WARN)
  { "_warn", (FAR const void *)_warn },
#endif
  { "abort", (FAR const void *)&abort },
  { "abs", (FAR const void *)abs },
#if defined(CONFIG_FS_AIO)
  { "aio_error", (FAR const void *)aio_error },
#endif
#if defined(CONFIG_FS_AIO)
  { "aio_return", (FAR const void *)aio_return },
#endif
#if defined(CONFIG_FS_AIO)
  { "aio_suspend", (FAR const void *)aio_suspend },
#endif
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "alarm", (FAR const void *)alarm },
#endif
  { "alphasort", (FAR const void *)alphasort },
  { "arc4random", (FAR const void *)&arc4random },
  { "arc4random_buf", (FAR const void *)arc4random_buf },
  { "asprintf", (FAR const void *)asprintf },
#if defined(CONFIG_HAVE_DOUBLE)
  { "atof", (FAR const void *)atof },
#endif
  { "atoi", (FAR const void *)atoi },
  { "atol", (FAR const void *)atol },
#if defined(CONFIG_HAVE_LONG_LONG)
  { "atoll", (FAR const void *)atoll },
#endif
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "b16atan2", (FAR const void *)b16atan2 },
#endif
  { "b16cos", (FAR const void *)b16cos },
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "b16divb16", (FAR const void *)b16divb16 },
#endif
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "b16mulb16", (FAR const void *)b16mulb16 },
#endif
  { "b16sin", (FAR const void *)b16sin },
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "b16sqr", (FAR const void *)b16sqr },
#endif
  { "basename", (FAR const void *)basename },
#if defined(CONFIG_LIBC_LOCALE_GETTEXT)
  { "bind_textdomain_codeset", (FAR const void *)bind_textdomain_codeset },
#endif
#if defined(CONFIG_LIBC_LOCALE_GETTEXT)
  { "bindtextdomain", (FAR const void *)bindtextdomain },
#endif
  { "bsearch", (FAR const void *)bsearch },
  { "btowc", (FAR const void *)btowc },
  { "calloc", (FAR const void *)calloc },
  { "cfgetspeed", (FAR const void *)cfgetspeed },
  { "cfsetspeed", (FAR const void *)cfsetspeed },
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "chdir", (FAR const void *)chdir },
#endif
  { "clock_getcpuclockid", (FAR const void *)clock_getcpuclockid },
  { "clock_getres", (FAR const void *)clock_getres },
  { "closedir", (FAR const void *)closedir },
  { "crc32", (FAR const void *)crc32 },
  { "crc32part", (FAR const void *)crc32part },
  { "ctime", (FAR const void *)ctime },
#if defined(CONFIG_CRYPTO)
  { "crypt", (FAR const void *)crypt },
#endif
#if defined(CONFIG_CRYPTO)
  { "crypt_r", (FAR const void *)crypt_r },
#endif
  { "daemon", (FAR const void *)daemon },
#if defined(CONFIG_LIBC_LOCALE_GETTEXT)
  { "dgettext", (FAR const void *)dgettext },
#endif
  { "dirname", (FAR const void *)dirname },
#if defined(CONFIG_LIBC_DLFCN)
  { "dlclose", (FAR const void *)dlclose },
#endif
#if defined(CONFIG_LIBC_DLFCN)
  { "dlerror", (FAR const void *)&dlerror },
#endif
#if defined(CONFIG_LIBC_DLFCN)
  { "dlopen", (FAR const void *)dlopen },
#endif
#if defined(CONFIG_LIBC_DLFCN)
  { "dlsym", (FAR const void *)dlsym },
#endif
#if defined(CONFIG_LIBC_DLFCN)
  { "dlsymtab", (FAR const void *)dlsymtab },
#endif
  { "dq_addafter", (FAR const void *)dq_addafter },
  { "dq_remfirst", (FAR const void *)dq_remfirst },
  { "dq_remlast", (FAR const void *)dq_remlast },
  { "ether_ntoa", (FAR const void *)ether_ntoa },
#if defined(CONFIG_LIBC_EXECFUNCS)
  { "execv", (FAR const void *)execv },
#endif
  { "exit", (FAR const void *)exit },
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "fchdir", (FAR const void *)fchdir },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fclose", (FAR const void *)fclose },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fdopen", (FAR const void *)fdopen },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "feof", (FAR const void *)feof },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "ferror", (FAR const void *)ferror },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fflush", (FAR const void *)fflush },
#endif
  { "ffs", (FAR const void *)ffs },
#if defined(CONFIG_FILE_STREAM)
  { "fgetc", (FAR const void *)fgetc },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fgetpos", (FAR const void *)fgetpos },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fgets", (FAR const void *)fgets },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fgetwc", (FAR const void *)fgetwc },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fgetwc_unlocked", (FAR const void *)fgetwc_unlocked },
#endif
  { "fileno", (FAR const void *)fileno },
#if !defined(CONFIG_FILE_STREAM)
  { "flockfile", (FAR const void *)flockfile },
#endif
  { "fnmatch", (FAR const void *)fnmatch },
#if defined(CONFIG_FILE_STREAM)
  { "fopen", (FAR const void *)fopen },
#endif
#if !defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_ARCH_HAVE_FORK)
  { "fork", (FAR const void *)&fork },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fprintf", (FAR const void *)fprintf },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fputc", (FAR const void *)fputc },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fputs", (FAR const void *)fputs },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fputwc", (FAR const void *)fputwc },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fputwc_unlocked", (FAR const void *)fputwc_unlocked },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fputws", (FAR const void *)fputws },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fputws_unlocked", (FAR const void *)fputws_unlocked },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fread", (FAR const void *)fread },
#endif
  { "free", (FAR const void *)free },
#if defined(CONFIG_LIBC_NETDB)
  { "freeaddrinfo", (FAR const void *)freeaddrinfo },
#endif
  { "fscanf", (FAR const void *)fscanf },
#if defined(CONFIG_FILE_STREAM)
  { "fseek", (FAR const void *)fseek },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fsetpos", (FAR const void *)fsetpos },
#endif
  { "fstatvfs", (FAR const void *)fstatvfs },
#if defined(CONFIG_FILE_STREAM)
  { "ftell", (FAR const void *)ftell },
#endif
#if !defined(CONFIG_FILE_STREAM)
  { "ftrylockfile", (FAR const void *)ftrylockfile },
#endif
#if !defined(CONFIG_FILE_STREAM)
  { "funlockfile", (FAR const void *)funlockfile },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fwrite", (FAR const void *)fwrite },
#endif
#if defined(CONFIG_LIBC_NETDB)
  { "gai_strerror", (FAR const void *)gai_strerror },
#endif
#if defined(CONFIG_LIBC_NETDB)
  { "getaddrinfo", (FAR const void *)getaddrinfo },
#endif
  { "getc", (FAR const void *)getc },
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "getcwd", (FAR const void *)getcwd },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "get_current_dir_name", (FAR const void *)get_current_dir_name },
#endif
  { "getegid", (FAR const void *)&getegid },
  { "geteuid", (FAR const void *)&geteuid },
#if defined(CONFIG_LIBC_NETDB)
  { "gethostbyname", (FAR const void *)gethostbyname },
#endif
#if defined(CONFIG_LIBC_NETDB)
  { "gethostbyname2", (FAR const void *)gethostbyname2 },
#endif
  { "gethostname", (FAR const void *)gethostname },
#if defined(CONFIG_LIBC_NETDB)
  { "getnameinfo", (FAR const void *)getnameinfo },
#endif
  { "getopt", (FAR const void *)getopt },
  { "getoptargp", (FAR const void *)&getoptargp },
  { "getopterrp", (FAR const void *)&getopterrp },
  { "getoptindp", (FAR const void *)&getoptindp },
  { "getoptoptp", (FAR const void *)&getoptoptp },
  { "getpriority", (FAR const void *)getpriority },
  { "getpass", (FAR const void *)getpass },
  { "getpwnam_r", (FAR const void *)getpwnam_r },
  { "getpwuid_r", (FAR const void *)getpwuid_r },
#if !defined(CONFIG_BUILD_KERNEL)
  { "getrandom", (FAR const void *)getrandom },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "gets", (FAR const void *)gets },
#endif
#if defined(CONFIG_LIBC_LOCALE_GETTEXT)
  { "gettext", (FAR const void *)gettext },
#endif
  { "gettimeofday", (FAR const void *)gettimeofday },
#if defined(CONFIG_FILE_STREAM)
  { "getwc", (FAR const void *)getwc },
#endif
  { "gmtime", (FAR const void *)gmtime },
  { "gmtime_r", (FAR const void *)gmtime_r },
  { "htonl", (FAR const void *)htonl },
  { "htonq", (FAR const void *)htonq },
  { "htons", (FAR const void *)htons },
#if defined(CONFIG_LIBC_LOCALE)
  { "iconv", (FAR const void *)iconv },
#endif
#if defined(CONFIG_LIBC_LOCALE)
  { "iconv_close", (FAR const void *)iconv_close },
#endif
#if defined(CONFIG_LIBC_LOCALE)
  { "iconv_open", (FAR const void *)iconv_open },
#endif
  { "imaxabs", (FAR const void *)imaxabs },
  { "inet_addr", (FAR const void *)inet_addr },
#if defined(CONFIG_NET_IPv4)
  { "inet_ntoa", (FAR const void *)inet_ntoa },
#endif
  { "inet_ntop", (FAR const void *)inet_ntop },
  { "inet_pton", (FAR const void *)inet_pton },
  { "isalnum", (FAR const void *)isalnum },
  { "isalpha", (FAR const void *)isalpha },
  { "isascii", (FAR const void *)isascii },
  { "isatty", (FAR const void *)isatty },
  { "isblank", (FAR const void *)isblank },
  { "iscntrl", (FAR const void *)iscntrl },
  { "isdigit", (FAR const void *)isdigit },
  { "isgraph", (FAR const void *)isgraph },
  { "islower", (FAR const void *)islower },
  { "isprint", (FAR const void *)isprint },
  { "ispunct", (FAR const void *)ispunct },
  { "isspace", (FAR const void *)isspace },
  { "isupper", (FAR const void *)isupper },
  { "iswalnum", (FAR const void *)iswalnum },
  { "iswalpha", (FAR const void *)iswalpha },
  { "iswblank", (FAR const void *)iswblank },
  { "iswcntrl", (FAR const void *)iswcntrl },
  { "iswctype", (FAR const void *)iswctype },
  { "iswdigit", (FAR const void *)iswdigit },
  { "iswgraph", (FAR const void *)iswgraph },
  { "iswlower", (FAR const void *)iswlower },
  { "iswprint", (FAR const void *)iswprint },
  { "iswpunct", (FAR const void *)iswpunct },
  { "iswspace", (FAR const void *)iswspace },
  { "iswupper", (FAR const void *)iswupper },
  { "iswxdigit", (FAR const void *)iswxdigit },
  { "isxdigit", (FAR const void *)isxdigit },
  { "labs", (FAR const void *)labs },
  { "lib_dumpbuffer", (FAR const void *)lib_dumpbuffer },
  { "lib_get_stream", (FAR const void *)lib_get_stream },
#if defined(CONFIG_FS_AIO)
  { "lio_listio", (FAR const void *)lio_listio },
#endif
#if defined(CONFIG_HAVE_LONG_LONG)
  { "llabs", (FAR const void *)llabs },
#endif
  { "localtime", (FAR const void *)localtime },
  { "localtime_r", (FAR const void *)localtime_r },
  { "mallinfo", (FAR const void *)mallinfo },
  { "malloc", (FAR const void *)malloc },
  { "malloc_size", (FAR const void *)malloc_size },
  { "mblen", (FAR const void *)mblen },
  { "mbrlen", (FAR const void *)mbrlen },
  { "mbrtowc", (FAR const void *)mbrtowc },
  { "mbsnrtowcs", (FAR const void *)mbsnrtowcs },
  { "mbsrtowcs", (FAR const void *)mbsrtowcs },
  { "mbstowcs", (FAR const void *)mbstowcs },
  { "mbtowc", (FAR const void *)mbtowc },
  { "memccpy", (FAR const void *)memccpy },
  { "memchr", (FAR const void *)memchr },
  { "memcmp", (FAR const void *)memcmp },
  { "memcpy", (FAR const void *)memcpy },
  { "memmove", (FAR const void *)memmove },
  { "memset", (FAR const void *)memset },
  { "mkdtemp", (FAR const void *)mkdtemp },
#if defined(CONFIG_PIPES) && CONFIG_DEV_FIFO_SIZE > 0
  { "mkfifo", (FAR const void *)mkfifo },
#endif
  { "mkstemp", (FAR const void *)mkstemp },
  { "mktemp", (FAR const void *)mktemp },
  { "mktime", (FAR const void *)mktime },
  { "nice", (FAR const void *)nice },
  { "ntohl", (FAR const void *)ntohl },
  { "ntohq", (FAR const void *)ntohq },
  { "ntohs", (FAR const void *)ntohs },
  { "opendir", (FAR const void *)opendir },
#if defined(CONFIG_FILE_STREAM)
  { "perror", (FAR const void *)perror },
#endif
#if defined(CONFIG_PIPES) && CONFIG_DEV_PIPE_SIZE > 0
  { "pipe", (FAR const void *)pipe },
#endif
  { "posix_fallocate", (FAR const void *)posix_fallocate },
  { "posix_memalign", (FAR const void *)posix_memalign },
  { "preadv", (FAR const void *)preadv },
  { "printf", (FAR const void *)printf },
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_destroy", (FAR const void *)pthread_attr_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_getinheritsched", (FAR const void *)pthread_attr_getinheritsched },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_getschedparam", (FAR const void *)pthread_attr_getschedparam },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_getschedpolicy", (FAR const void *)pthread_attr_getschedpolicy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_getstacksize", (FAR const void *)pthread_attr_getstacksize },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_init", (FAR const void *)pthread_attr_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_setinheritsched", (FAR const void *)pthread_attr_setinheritsched },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_setschedparam", (FAR const void *)pthread_attr_setschedparam },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_setschedpolicy", (FAR const void *)pthread_attr_setschedpolicy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_setstacksize", (FAR const void *)pthread_attr_setstacksize },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrier_destroy", (FAR const void *)pthread_barrier_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrier_init", (FAR const void *)pthread_barrier_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrier_wait", (FAR const void *)pthread_barrier_wait },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrierattr_destroy", (FAR const void *)pthread_barrierattr_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrierattr_getpshared", (FAR const void *)pthread_barrierattr_getpshared },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrierattr_init", (FAR const void *)pthread_barrierattr_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrierattr_setpshared", (FAR const void *)pthread_barrierattr_setpshared },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_destroy", (FAR const void *)pthread_cond_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_init", (FAR const void *)pthread_cond_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_timedwait", (FAR const void *)pthread_cond_timedwait },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_condattr_destroy", (FAR const void *)pthread_condattr_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_condattr_init", (FAR const void *)pthread_condattr_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_condattr_setclock", (FAR const void *)pthread_condattr_setclock },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_create", (FAR const void *)pthread_create },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_getname_np", (FAR const void *)pthread_getname_np },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_TLS_NELEM > 0
  { "pthread_getspecific", (FAR const void *)pthread_getspecific },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_gettid_np", (FAR const void *)pthread_gettid_np },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_TLS_NELEM > 0
  { "pthread_key_create", (FAR const void *)pthread_key_create },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_TLS_NELEM > 0
  { "pthread_key_delete", (FAR const void *)pthread_key_delete },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_PRIORITY_PROTECT)
  { "pthread_mutex_getprioceiling", (FAR const void *)pthread_mutex_getprioceiling },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_lock", (FAR const void *)pthread_mutex_lock },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_PRIORITY_PROTECT)
  { "pthread_mutex_setprioceiling", (FAR const void *)pthread_mutex_setprioceiling },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutexattr_destroy", (FAR const void *)pthread_mutexattr_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_PRIORITY_PROTECT)
  { "pthread_mutexattr_getprioceiling", (FAR const void *)pthread_mutexattr_getprioceiling },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutexattr_getpshared", (FAR const void *)pthread_mutexattr_getpshared },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_PTHREAD_MUTEX_TYPES)
  { "pthread_mutexattr_gettype", (FAR const void *)pthread_mutexattr_gettype },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutexattr_init", (FAR const void *)pthread_mutexattr_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_PRIORITY_PROTECT)
  { "pthread_mutexattr_setprioceiling", (FAR const void *)pthread_mutexattr_setprioceiling },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutexattr_setpshared", (FAR const void *)pthread_mutexattr_setpshared },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_PTHREAD_MUTEX_TYPES)
  { "pthread_mutexattr_settype", (FAR const void *)pthread_mutexattr_settype },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_once", (FAR const void *)pthread_once },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_rwlock_destroy", (FAR const void *)pthread_rwlock_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_rwlock_init", (FAR const void *)pthread_rwlock_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_rwlock_rdlock", (FAR const void *)pthread_rwlock_rdlock },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_rwlock_unlock", (FAR const void *)pthread_rwlock_unlock },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_self", (FAR const void *)pthread_self },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_setname_np", (FAR const void *)pthread_setname_np },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_TLS_NELEM > 0
  { "pthread_setspecific", (FAR const void *)pthread_setspecific },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_yield", (FAR const void *)&pthread_yield },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "puts", (FAR const void *)puts },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "putwc", (FAR const void *)putwc },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "putwc_unlocked", (FAR const void *)putwc_unlocked },
#endif
  { "putwchar", (FAR const void *)putwchar },
  { "pwritev", (FAR const void *)pwritev },
  { "qsort", (FAR const void *)qsort },
  { "raise", (FAR const void *)raise },
  { "rand", (FAR const void *)&rand },
  { "readdir", (FAR const void *)readdir },
  { "readdir_r", (FAR const void *)readdir_r },
  { "readv", (FAR const void *)readv },
  { "realloc", (FAR const void *)realloc },
  { "remove", (FAR const void *)remove },
#if defined(CONFIG_FILE_STREAM)
  { "rewind", (FAR const void *)rewind },
#endif
  { "scandir", (FAR const void *)scandir },
  { "sched_get_priority_max", (FAR const void *)sched_get_priority_max },
  { "sched_get_priority_min", (FAR const void *)sched_get_priority_min },
  { "sem_getvalue", (FAR const void *)sem_getvalue },
  { "sem_init", (FAR const void *)sem_init },
#if defined(CONFIG_LIBC_LOCALE)
  { "setlocale", (FAR const void *)setlocale },
#endif
  { "setlogmask", (FAR const void *)setlogmask },
  { "setpriority", (FAR const void *)setpriority },
#if defined(CONFIG_NET)
  { "shutdown", (FAR const void *)shutdown },
#endif
  { "sigaddset", (FAR const void *)sigaddset },
  { "sigdelset", (FAR const void *)sigdelset },
  { "sigemptyset", (FAR const void *)sigemptyset },
  { "sigfillset", (FAR const void *)sigfillset },
  { "sigismember", (FAR const void *)sigismember },
  { "signal", (FAR const void *)signal },
  { "sleep", (FAR const void *)sleep },
  { "snprintf", (FAR const void *)snprintf },
  { "sprintf", (FAR const void *)sprintf },
  { "sprintf", (FAR const void *)sprintf },
  { "sq_addafter", (FAR const void *)sq_addafter },
  { "sq_remafter", (FAR const void *)sq_remafter },
  { "sq_remfirst", (FAR const void *)sq_remfirst },
  { "sq_remlast", (FAR const void *)sq_remlast },
  { "srand", (FAR const void *)srand },
  { "sscanf", (FAR const void *)sscanf },
  { "sscanf", (FAR const void *)sscanf },
  { "statvfs", (FAR const void *)statvfs },
  { "stpcpy", (FAR const void *)stpcpy },
  { "strcasecmp", (FAR const void *)strcasecmp },
  { "strcasestr", (FAR const void *)strcasestr },
  { "strcat", (FAR const void *)strcat },
  { "strchr", (FAR const void *)strchr },
  { "strchrnul", (FAR const void *)strchrnul },
  { "strcmp", (FAR const void *)strcmp },
#if defined(CONFIG_LIBC_LOCALE)
  { "strcoll", (FAR const void *)strcoll },
#endif
  { "strcpy", (FAR const void *)strcpy },
  { "strcspn", (FAR const void *)strcspn },
  { "strdup", (FAR const void *)strdup },
  { "strerror", (FAR const void *)strerror },
  { "strerror_r", (FAR const void *)strerror_r },
  { "strftime", (FAR const void *)strftime },
  { "strlcpy", (FAR const void *)strlcpy },
  { "strlen", (FAR const void *)strlen },
  { "strncasecmp", (FAR const void *)strncasecmp },
  { "strncat", (FAR const void *)strncat },
  { "strncmp", (FAR const void *)strncmp },
  { "strncpy", (FAR const void *)strncpy },
  { "strndup", (FAR const void *)strndup },
  { "strnlen", (FAR const void *)strnlen },
  { "strpbrk", (FAR const void *)strpbrk },
  { "strrchr", (FAR const void *)strrchr },
  { "strsep", (FAR const void *)strsep },
  { "strspn", (FAR const void *)strspn },
  { "strstr", (FAR const void *)strstr },
  { "strtod", (FAR const void *)strtod },
#if defined(CONFIG_HAVE_DOUBLE)
  { "strtod", (FAR const void *)strtod },
#endif
  { "strtoimax", (FAR const void *)strtoimax },
  { "strtok", (FAR const void *)strtok },
  { "strtok_r", (FAR const void *)strtok_r },
  { "strtol", (FAR const void *)strtol },
#if defined(CONFIG_HAVE_LONG_LONG)
  { "strtoll", (FAR const void *)strtoll },
#endif
  { "strtoul", (FAR const void *)strtoul },
  { "strtoull", (FAR const void *)strtoull },
#if defined(CONFIG_HAVE_LONG_LONG)
  { "strtoull", (FAR const void *)strtoull },
#endif
  { "strtoumax", (FAR const void *)strtoumax },
#if defined(CONFIG_LIBC_LOCALE)
  { "strxfrm", (FAR const void *)strxfrm },
#endif
  { "swab", (FAR const void *)swab },
  { "swprintf", (FAR const void *)swprintf },
  { "sysconf", (FAR const void *)sysconf },
  { "syslog", (FAR const void *)syslog },
#if defined(CONFIG_CANCELLATION_POINTS)
  { "task_testcancel", (FAR const void *)&task_testcancel },
#endif
#if !defined(CONFIG_BUILD_KERNEL) && CONFIG_TLS_TASK_NELEM > 0
  { "task_tls_alloc", (FAR const void *)task_tls_alloc },
#endif
#if CONFIG_TLS_TASK_NELEM > 0
  { "task_tls_get_value", (FAR const void *)task_tls_get_value },
#endif
#if CONFIG_TLS_TASK_NELEM > 0
  { "task_tls_set_value", (FAR const void *)task_tls_set_value },
#endif
  { "tcflush", (FAR const void *)tcflush },
  { "tcgetattr", (FAR const void *)tcgetattr },
  { "tcsetattr", (FAR const void *)tcsetattr },
  { "telldir", (FAR const void *)telldir },
#if defined(CONFIG_LIBC_LOCALE_GETTEXT)
  { "textdomain", (FAR const void *)textdomain },
#endif
  { "time", (FAR const void *)time },
  { "tolower", (FAR const void *)tolower },
  { "toupper", (FAR const void *)toupper },
  { "towlower", (FAR const void *)towlower },
  { "towupper", (FAR const void *)towupper },
#if !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "truncate", (FAR const void *)truncate },
#endif
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "ub16divub16", (FAR const void *)ub16divub16 },
#endif
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "ub16mulub16", (FAR const void *)ub16mulub16 },
#endif
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "ub16sqr", (FAR const void *)ub16sqr },
#endif
  { "uname", (FAR const void *)uname },
#if defined(CONFIG_FILE_STREAM)
  { "ungetc", (FAR const void *)ungetc },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "ungetwc", (FAR const void *)ungetwc },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "ungetwc_unlocked", (FAR const void *)ungetwc_unlocked },
#endif
  { "usleep", (FAR const void *)usleep },
  { "vasprintf", (FAR const void *)vasprintf },
  { "versionsort", (FAR const void *)versionsort },
#if defined(CONFIG_FILE_STREAM)
  { "vfprintf", (FAR const void *)vfprintf },
#endif
  { "vprintf", (FAR const void *)vprintf },
#if defined(CONFIG_FILE_STREAM)
  { "vscanf", (FAR const void *)vscanf },
#endif
  { "vsnprintf", (FAR const void *)vsnprintf },
  { "vsprintf", (FAR const void *)vsprintf },
  { "vsscanf", (FAR const void *)vsscanf },
  { "vsyslog", (FAR const void *)vsyslog },
  { "wcrtomb", (FAR const void *)wcrtomb },
  { "wcscat", (FAR const void *)wcscat },
  { "wcschr", (FAR const void *)wcschr },
  { "wcscmp", (FAR const void *)wcscmp },
  { "wcscoll", (FAR const void *)wcscoll },
  { "wcscpy", (FAR const void *)wcscpy },
  { "wcscspn", (FAR const void *)wcscspn },
  { "wcsftime", (FAR const void *)wcsftime },
  { "wcslcat", (FAR const void *)wcslcat },
  { "wcsncat", (FAR const void *)wcsncat },
  { "wcsncmp", (FAR const void *)wcsncmp },
  { "wcsncpy", (FAR const void *)wcsncpy },
  { "wcslcpy", (FAR const void *)wcslcpy },
  { "wcslen", (FAR const void *)wcslen },
  { "wcsnrtombs", (FAR const void *)wcsnrtombs },
  { "wcspbrk", (FAR const void *)wcspbrk },
  { "wcsrchr", (FAR const void *)wcsrchr },
  { "wcsspn", (FAR const void *)wcsspn },
  { "wcsstr", (FAR const void *)wcsstr },
  { "wcsrtombs", (FAR const void *)wcsrtombs },
  { "wcstod", (FAR const void *)wcstod },
  { "wcstof", (FAR const void *)wcstof },
  { "wcstok", (FAR const void *)wcstok },
  { "wcstol", (FAR const void *)wcstol },
  { "wcstold", (FAR const void *)wcstold },
  { "wcstoll", (FAR const void *)wcstoll },
  { "wcstombs", (FAR const void *)wcstombs },
  { "wcstoul", (FAR const void *)wcstoul },
  { "wcswcs", (FAR const void *)wcswcs },
  { "wcswidth", (FAR const void *)wcswidth },
  { "wcsxfrm", (FAR const void *)wcsxfrm },
  { "wctob", (FAR const void *)wctob },
  { "wctomb", (FAR const void *)wctomb },
  { "wctype", (FAR const void *)wctype },
  { "wcwidth", (FAR const void *)wcwidth },
  { "wmemchr", (FAR const void *)wmemchr },
  { "wmemcmp", (FAR const void *)wmemcmp },
  { "wmemcpy", (FAR const void *)wmemcpy },
  { "wmemmove", (FAR const void *)wmemmove },
  { "wmemset", (FAR const void *)wmemset },
  { "writev", (FAR const void *)writev }
};

#define NSYMBOLS (sizeof(g_symtab) / sizeof (struct symtab_s))
int g_nsymbols = NSYMBOLS;
