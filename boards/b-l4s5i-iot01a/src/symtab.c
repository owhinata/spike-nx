/* boards/b-l4s5i-iot01a/src/symtab.c: Auto-generated symbol table + manual additions */

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/symtab.h>
#include <nuttx/clock.h>

/* ARM EABI helpers from libgcc */

extern void __aeabi_f2d(void);
extern void __aeabi_uldivmod(void);
extern void __aeabi_unwind_cpp_pr0(void);
extern void __aeabi_unwind_cpp_pr1(void);

#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <ssp/ssp.h>
#include <debug.h>
#include <unistd.h>
#include <sys/socket.h>
#include <math.h>
#include <sys/time.h>
#include <aio.h>
#include <dirent.h>
#include <stdio.h>
#include <fixedmath.h>
#include <libgen.h>
#include <libintl.h>
#include <sys/boardctl.h>
#include <wchar.h>
#include <termios.h>
#include <sys/stat.h>
#include <time.h>
#include <nuttx/crc32.h>
#include <dlfcn.h>
#include <nuttx/queue.h>
#include <sys/epoll.h>
#include <netinet/ether.h>
#include <sys/eventfd.h>
#include <nuttx/binfmt/binfmt.h>
#include <fcntl.h>
#include <strings.h>
#include <fnmatch.h>
#include <netdb.h>
#include <sys/statfs.h>
#include <sys/statvfs.h>
#include <sys/resource.h>
#include <pwd.h>
#include <sys/random.h>
#include <arpa/inet.h>
#include <iconv.h>
#include <inttypes.h>
#include <sys/inotify.h>
#include <nuttx/module.h>
#include <sys/ioctl.h>
#include <ctype.h>
#include <wctype.h>
#include <signal.h>
#include <nuttx/tls.h>
#include <malloc.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <mqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/pthread.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/arch.h>
#include <poll.h>
#include <spawn.h>
#include <sys/prctl.h>
#include <sys/uio.h>
#include <sys/select.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <sys/sendfile.h>
#include <locale.h>
#include <syslog.h>
#include <sys/shm.h>
#include <sys/signalfd.h>
#include <sys/sysinfo.h>
#include <nuttx/spawn.h>
#include <sys/timerfd.h>
#include <sys/utsname.h>
#include <sys/wait.h>

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
  { "_assert", (FAR const void *)_assert },
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG_ERROR)
  { "_err", (FAR const void *)_err },
#endif
  { "_exit", (FAR const void *)_exit },
  { "_exit", (FAR const void *)_exit },
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG_INFO)
  { "_info", (FAR const void *)_info },
#endif
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG_WARN)
  { "_warn", (FAR const void *)_warn },
#endif
  { "abort", (FAR const void *)&abort },
  { "abs", (FAR const void *)abs },
#if defined(CONFIG_NET)
  { "accept4", (FAR const void *)accept4 },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "acos", (FAR const void *)acos },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "acosf", (FAR const void *)acosf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "acosl", (FAR const void *)acosl },
#endif
#if defined(CONFIG_CLOCK_TIMEKEEPING)
  { "adjtime", (FAR const void *)adjtime },
#endif
#if defined(CONFIG_FS_AIO)
  { "aio_cancel", (FAR const void *)aio_cancel },
#endif
#if defined(CONFIG_FS_AIO)
  { "aio_error", (FAR const void *)aio_error },
#endif
#if defined(CONFIG_FS_AIO)
  { "aio_fsync", (FAR const void *)aio_fsync },
#endif
#if defined(CONFIG_FS_AIO)
  { "aio_read", (FAR const void *)aio_read },
#endif
#if defined(CONFIG_FS_AIO)
  { "aio_return", (FAR const void *)aio_return },
#endif
#if defined(CONFIG_FS_AIO)
  { "aio_suspend", (FAR const void *)aio_suspend },
#endif
#if defined(CONFIG_FS_AIO)
  { "aio_write", (FAR const void *)aio_write },
#endif
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "alarm", (FAR const void *)alarm },
#endif
  { "alphasort", (FAR const void *)alphasort },
  { "arc4random_buf", (FAR const void *)arc4random_buf },
  { "arc4random", (FAR const void *)&arc4random },
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "asin", (FAR const void *)asin },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "asinf", (FAR const void *)asinf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "asinl", (FAR const void *)asinl },
#endif
  { "asprintf", (FAR const void *)asprintf },
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "atan", (FAR const void *)atan },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "atan2", (FAR const void *)atan2 },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "atan2f", (FAR const void *)atan2f },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "atan2l", (FAR const void *)atan2l },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "atanf", (FAR const void *)atanf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "atanl", (FAR const void *)atanl },
#endif
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
#if defined(CONFIG_NET)
  { "bind", (FAR const void *)bind },
#endif
#if defined(CONFIG_LIBC_LOCALE_GETTEXT)
  { "bindtextdomain", (FAR const void *)bindtextdomain },
#endif
#if defined(CONFIG_BOARDCTL)
  { "boardctl", (FAR const void *)boardctl },
#endif
  { "bsearch", (FAR const void *)bsearch },
  { "btowc", (FAR const void *)btowc },
  { "calloc", (FAR const void *)calloc },
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "ceil", (FAR const void *)ceil },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "ceilf", (FAR const void *)ceilf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "ceill", (FAR const void *)ceill },
#endif
  { "cfgetspeed", (FAR const void *)cfgetspeed },
  { "cfsetspeed", (FAR const void *)cfsetspeed },
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "chdir", (FAR const void *)chdir },
#endif
  { "chmod", (FAR const void *)chmod },
  { "chown", (FAR const void *)chown },
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "clearenv", (FAR const void *)&clearenv },
#endif
  { "clock_getcpuclockid", (FAR const void *)clock_getcpuclockid },
  { "clock_getres", (FAR const void *)clock_getres },
  { "clock_gettime", (FAR const void *)clock_gettime },
  { "clock_nanosleep", (FAR const void *)clock_nanosleep },
  { "clock_settime", (FAR const void *)clock_settime },
  { "clock", (FAR const void *)&clock },
  { "close", (FAR const void *)close },
  { "closedir", (FAR const void *)closedir },
#if defined(CONFIG_NET)
  { "connect", (FAR const void *)connect },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "cos", (FAR const void *)cos },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "cosf", (FAR const void *)cosf },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "cosh", (FAR const void *)cosh },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "coshf", (FAR const void *)coshf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "coshl", (FAR const void *)coshl },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "cosl", (FAR const void *)cosl },
#endif
  { "crc32", (FAR const void *)crc32 },
  { "crc32part", (FAR const void *)crc32part },
#if defined(CONFIG_CRYPTO)
  { "crypt_r", (FAR const void *)crypt_r },
#endif
#if defined(CONFIG_CRYPTO)
  { "crypt", (FAR const void *)crypt },
#endif
  { "ctime", (FAR const void *)ctime },
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
  { "dup", (FAR const void *)dup },
  { "dup2", (FAR const void *)dup2 },
  { "epoll_close", (FAR const void *)epoll_close },
  { "epoll_create", (FAR const void *)epoll_create },
  { "epoll_create1", (FAR const void *)epoll_create1 },
  { "epoll_ctl", (FAR const void *)epoll_ctl },
  { "epoll_pwait", (FAR const void *)epoll_pwait },
  { "epoll_wait", (FAR const void *)epoll_wait },
  { "ether_ntoa", (FAR const void *)ether_ntoa },
#if defined(CONFIG_EVENT_FD)
  { "eventfd", (FAR const void *)eventfd },
#endif
#if !defined(CONFIG_BINFMT_DISABLE) && !defined(CONFIG_BUILD_KERNEL)
  { "exec", (FAR const void *)exec },
#endif
#if defined(CONFIG_LIBC_EXECFUNCS)
  { "execv", (FAR const void *)execv },
#endif
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_LIBC_EXECFUNCS)
  { "execve", (FAR const void *)execve },
#endif
  { "exit", (FAR const void *)exit },
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "exp", (FAR const void *)exp },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "expf", (FAR const void *)expf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "expl", (FAR const void *)expl },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "fabs", (FAR const void *)fabs },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "fabsf", (FAR const void *)fabsf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "fabsl", (FAR const void *)fabsl },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "fchdir", (FAR const void *)fchdir },
#endif
  { "fchmod", (FAR const void *)fchmod },
  { "fchown", (FAR const void *)fchown },
#if defined(CONFIG_FILE_STREAM)
  { "fclose", (FAR const void *)fclose },
#endif
  { "fcntl", (FAR const void *)fcntl },
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
  { "fgetwc_unlocked", (FAR const void *)fgetwc_unlocked },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fgetwc", (FAR const void *)fgetwc },
#endif
  { "fileno", (FAR const void *)fileno },
#if !defined(CONFIG_FILE_STREAM)
  { "flockfile", (FAR const void *)flockfile },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "floor", (FAR const void *)floor },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "floorf", (FAR const void *)floorf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "floorl", (FAR const void *)floorl },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "fmod", (FAR const void *)fmod },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "fmodf", (FAR const void *)fmodf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "fmodl", (FAR const void *)fmodl },
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
  { "fputwc_unlocked", (FAR const void *)fputwc_unlocked },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fputwc", (FAR const void *)fputwc },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fputws_unlocked", (FAR const void *)fputws_unlocked },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fputws", (FAR const void *)fputws },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fread", (FAR const void *)fread },
#endif
  { "free", (FAR const void *)free },
#if defined(CONFIG_LIBC_NETDB)
  { "freeaddrinfo", (FAR const void *)freeaddrinfo },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "frexp", (FAR const void *)frexp },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "frexpf", (FAR const void *)frexpf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "frexpl", (FAR const void *)frexpl },
#endif
  { "fscanf", (FAR const void *)fscanf },
#if defined(CONFIG_FILE_STREAM)
  { "fseek", (FAR const void *)fseek },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "fsetpos", (FAR const void *)fsetpos },
#endif
  { "fstat", (FAR const void *)fstat },
  { "fstatfs", (FAR const void *)fstatfs },
  { "fstatvfs", (FAR const void *)fstatvfs },
  { "fsync", (FAR const void *)fsync },
#if defined(CONFIG_FILE_STREAM)
  { "ftell", (FAR const void *)ftell },
#endif
  { "ftruncate", (FAR const void *)ftruncate },
#if !defined(CONFIG_FILE_STREAM)
  { "ftrylockfile", (FAR const void *)ftrylockfile },
#endif
#if !defined(CONFIG_FILE_STREAM)
  { "funlockfile", (FAR const void *)funlockfile },
#endif
  { "futimens", (FAR const void *)futimens },
#if defined(CONFIG_FILE_STREAM)
  { "fwrite", (FAR const void *)fwrite },
#endif
#if defined(CONFIG_LIBC_NETDB)
  { "gai_strerror", (FAR const void *)gai_strerror },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "gamma", (FAR const void *)gamma },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "get_current_dir_name", (FAR const void *)get_current_dir_name },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "get_environ_ptr", (FAR const void *)&get_environ_ptr },
#endif
#if defined(CONFIG_LIBC_NETDB)
  { "getaddrinfo", (FAR const void *)getaddrinfo },
#endif
  { "getc", (FAR const void *)getc },
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "getcwd", (FAR const void *)getcwd },
#endif
  { "getegid", (FAR const void *)&getegid },
#if defined(CONFIG_SCHED_USER_IDENTITY)
  { "getegid", (FAR const void *)&getegid },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "getenv", (FAR const void *)getenv },
#endif
  { "geteuid", (FAR const void *)&geteuid },
#if defined(CONFIG_SCHED_USER_IDENTITY)
  { "geteuid", (FAR const void *)&geteuid },
#endif
#if defined(CONFIG_SCHED_USER_IDENTITY)
  { "getgid", (FAR const void *)&getgid },
#endif
#if defined(CONFIG_LIBC_NETDB)
  { "gethostbyname", (FAR const void *)gethostbyname },
#endif
#if defined(CONFIG_LIBC_NETDB)
  { "gethostbyname2", (FAR const void *)gethostbyname2 },
#endif
  { "gethostname", (FAR const void *)gethostname },
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "getitimer", (FAR const void *)getitimer },
#endif
#if defined(CONFIG_LIBC_NETDB)
  { "getnameinfo", (FAR const void *)getnameinfo },
#endif
  { "getopt", (FAR const void *)getopt },
  { "getoptargp", (FAR const void *)&getoptargp },
  { "getopterrp", (FAR const void *)&getopterrp },
  { "getoptindp", (FAR const void *)&getoptindp },
  { "getoptoptp", (FAR const void *)&getoptoptp },
  { "getpass", (FAR const void *)getpass },
#if defined(CONFIG_NET)
  { "getpeername", (FAR const void *)getpeername },
#endif
  { "getpid", (FAR const void *)&getpid },
#if defined(CONFIG_SCHED_HAVE_PARENT)
  { "getppid", (FAR const void *)&getppid },
#endif
  { "getpriority", (FAR const void *)getpriority },
  { "getpwnam_r", (FAR const void *)getpwnam_r },
  { "getpwuid_r", (FAR const void *)getpwuid_r },
#if !defined(CONFIG_BUILD_KERNEL)
  { "getrandom", (FAR const void *)getrandom },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "gets", (FAR const void *)gets },
#endif
#if defined(CONFIG_NET)
  { "getsockname", (FAR const void *)getsockname },
#endif
#if defined(CONFIG_NET)
  { "getsockopt", (FAR const void *)getsockopt },
#endif
#if defined(CONFIG_LIBC_LOCALE_GETTEXT)
  { "gettext", (FAR const void *)gettext },
#endif
  { "gettimeofday", (FAR const void *)gettimeofday },
#if defined(CONFIG_SCHED_USER_IDENTITY)
  { "getuid", (FAR const void *)&getuid },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "getwc", (FAR const void *)getwc },
#endif
  { "gmtime_r", (FAR const void *)gmtime_r },
  { "gmtime", (FAR const void *)gmtime },
  { "htonl", (FAR const void *)htonl },
  { "htonq", (FAR const void *)htonq },
  { "htons", (FAR const void *)htons },
#if defined(CONFIG_LIBC_LOCALE)
  { "iconv_close", (FAR const void *)iconv_close },
#endif
#if defined(CONFIG_LIBC_LOCALE)
  { "iconv_open", (FAR const void *)iconv_open },
#endif
#if defined(CONFIG_LIBC_LOCALE)
  { "iconv", (FAR const void *)iconv },
#endif
  { "imaxabs", (FAR const void *)imaxabs },
  { "inet_addr", (FAR const void *)inet_addr },
#if defined(CONFIG_NET_IPv4)
  { "inet_ntoa", (FAR const void *)inet_ntoa },
#endif
  { "inet_ntop", (FAR const void *)inet_ntop },
  { "inet_pton", (FAR const void *)inet_pton },
#if defined(CONFIG_FS_NOTIFY)
  { "inotify_add_watch", (FAR const void *)inotify_add_watch },
#endif
#if defined(CONFIG_FS_NOTIFY)
  { "inotify_init", (FAR const void *)&inotify_init },
#endif
#if defined(CONFIG_FS_NOTIFY)
  { "inotify_init1", (FAR const void *)inotify_init1 },
#endif
#if defined(CONFIG_FS_NOTIFY)
  { "inotify_rm_watch", (FAR const void *)inotify_rm_watch },
#endif
#if defined(CONFIG_MODULE)
  { "insmod", (FAR const void *)insmod },
#endif
  { "ioctl", (FAR const void *)ioctl },
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
  { "kill", (FAR const void *)kill },
  { "labs", (FAR const void *)labs },
  { "lchmod", (FAR const void *)lchmod },
  { "lchown", (FAR const void *)lchown },
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "ldexp", (FAR const void *)ldexp },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "ldexpf", (FAR const void *)ldexpf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "ldexpl", (FAR const void *)ldexpl },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "lgamma", (FAR const void *)lgamma },
#endif
  { "lib_dumpbuffer", (FAR const void *)lib_dumpbuffer },
  { "lib_get_stream", (FAR const void *)lib_get_stream },
#if defined(CONFIG_PSEUDOFS_SOFTLINKS)
  { "link", (FAR const void *)link },
#endif
#if defined(CONFIG_FS_AIO)
  { "lio_listio", (FAR const void *)lio_listio },
#endif
#if defined(CONFIG_NET)
  { "listen", (FAR const void *)listen },
#endif
#if defined(CONFIG_HAVE_LONG_LONG)
  { "llabs", (FAR const void *)llabs },
#endif
  { "localtime_r", (FAR const void *)localtime_r },
  { "localtime", (FAR const void *)localtime },
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "log", (FAR const void *)log },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "log10", (FAR const void *)log10 },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "log10f", (FAR const void *)log10f },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "log10l", (FAR const void *)log10l },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "log2", (FAR const void *)log2 },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "log2f", (FAR const void *)log2f },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "log2l", (FAR const void *)log2l },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "logf", (FAR const void *)logf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "logl", (FAR const void *)logl },
#endif
  { "lseek", (FAR const void *)lseek },
  { "lstat", (FAR const void *)lstat },
  { "lutimens", (FAR const void *)lutimens },
  { "mallinfo", (FAR const void *)mallinfo },
  { "malloc_size", (FAR const void *)malloc_size },
  { "malloc", (FAR const void *)malloc },
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
#if !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "mkdir", (FAR const void *)mkdir },
#endif
  { "mkdtemp", (FAR const void *)mkdtemp },
#if defined(CONFIG_PIPES) && CONFIG_DEV_FIFO_SIZE > 0
  { "mkfifo", (FAR const void *)mkfifo },
#endif
  { "mkstemp", (FAR const void *)mkstemp },
  { "mktemp", (FAR const void *)mktemp },
  { "mktime", (FAR const void *)mktime },
  { "mmap", (FAR const void *)mmap },
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "modf", (FAR const void *)modf },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "modff", (FAR const void *)modff },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "modfl", (FAR const void *)modfl },
#endif
#if defined(CONFIG_MODULE)
  { "modhandle", (FAR const void *)modhandle },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "mount", (FAR const void *)mount },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_close", (FAR const void *)mq_close },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_getattr", (FAR const void *)mq_getattr },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_notify", (FAR const void *)mq_notify },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_open", (FAR const void *)mq_open },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_receive", (FAR const void *)mq_receive },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_send", (FAR const void *)mq_send },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_setattr", (FAR const void *)mq_setattr },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_timedreceive", (FAR const void *)mq_timedreceive },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_timedsend", (FAR const void *)mq_timedsend },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_unlink", (FAR const void *)mq_unlink },
#endif
  { "msync", (FAR const void *)msync },
  { "munmap", (FAR const void *)munmap },
  { "nanosleep", (FAR const void *)nanosleep },
  { "nice", (FAR const void *)nice },
  { "ntohl", (FAR const void *)ntohl },
  { "ntohq", (FAR const void *)ntohq },
  { "ntohs", (FAR const void *)ntohs },
#if defined(CONFIG_PIPES) && CONFIG_DEV_FIFO_SIZE > 0
  { "nx_mkfifo", (FAR const void *)nx_mkfifo },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "nx_pthread_create", (FAR const void *)nx_pthread_create },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "nx_pthread_exit", (FAR const void *)nx_pthread_exit },
#endif
  { "nx_vsyslog", (FAR const void *)nx_vsyslog },
  { "nxsched_get_stackinfo", (FAR const void *)nxsched_get_stackinfo },
  { "nxsem_clockwait", (FAR const void *)nxsem_clockwait },
#if defined(CONFIG_FS_NAMED_SEMAPHORES)
  { "nxsem_close", (FAR const void *)nxsem_close },
#endif
  { "nxsem_destroy", (FAR const void *)nxsem_destroy },
#if defined(CONFIG_PRIORITY_PROTECT)
  { "nxsem_getprioceiling", (FAR const void *)nxsem_getprioceiling },
#endif
#if defined(CONFIG_FS_NAMED_SEMAPHORES)
  { "nxsem_open", (FAR const void *)nxsem_open },
#endif
  { "nxsem_post_slow", (FAR const void *)nxsem_post_slow },
#if defined(CONFIG_PRIORITY_INHERITANCE)
  { "nxsem_set_protocol", (FAR const void *)nxsem_set_protocol },
#endif
#if defined(CONFIG_PRIORITY_PROTECT)
  { "nxsem_setprioceiling", (FAR const void *)nxsem_setprioceiling },
#endif
  { "nxsem_tickwait", (FAR const void *)nxsem_tickwait },
  { "nxsem_timedwait", (FAR const void *)nxsem_timedwait },
  { "nxsem_trywait_slow", (FAR const void *)nxsem_trywait_slow },
#if defined(CONFIG_FS_NAMED_SEMAPHORES)
  { "nxsem_unlink", (FAR const void *)nxsem_unlink },
#endif
  { "nxsem_wait_slow", (FAR const void *)nxsem_wait_slow },
  { "open", (FAR const void *)open },
  { "opendir", (FAR const void *)opendir },
#if defined(CONFIG_FILE_STREAM)
  { "perror", (FAR const void *)perror },
#endif
#if defined(CONFIG_BUILD_KERNEL)
  { "pgalloc", (FAR const void *)pgalloc },
#endif
#if defined(CONFIG_PIPES) && CONFIG_DEV_PIPE_SIZE > 0
  { "pipe", (FAR const void *)pipe },
#endif
#if defined(CONFIG_PIPES) && CONFIG_DEV_PIPE_SIZE > 0
  { "pipe2", (FAR const void *)pipe2 },
#endif
  { "poll", (FAR const void *)poll },
  { "posix_fallocate", (FAR const void *)posix_fallocate },
  { "posix_memalign", (FAR const void *)posix_memalign },
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_LIBC_EXECFUNCS)
  { "posix_spawn", (FAR const void *)posix_spawn },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "pow", (FAR const void *)pow },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "powf", (FAR const void *)powf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "powl", (FAR const void *)powl },
#endif
  { "ppoll", (FAR const void *)ppoll },
  { "prctl", (FAR const void *)prctl },
  { "pread", (FAR const void *)pread },
  { "preadv", (FAR const void *)preadv },
  { "printf", (FAR const void *)printf },
  { "pselect", (FAR const void *)pselect },
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
  { "pthread_cancel", (FAR const void *)pthread_cancel },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_broadcast", (FAR const void *)pthread_cond_broadcast },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_clockwait", (FAR const void *)pthread_cond_clockwait },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_destroy", (FAR const void *)pthread_cond_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_init", (FAR const void *)pthread_cond_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_signal", (FAR const void *)pthread_cond_signal },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_timedwait", (FAR const void *)pthread_cond_timedwait },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_wait", (FAR const void *)pthread_cond_wait },
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
  { "pthread_detach", (FAR const void *)pthread_detach },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_SMP)
  { "pthread_getaffinity_np", (FAR const void *)pthread_getaffinity_np },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_getname_np", (FAR const void *)pthread_getname_np },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_getschedparam", (FAR const void *)pthread_getschedparam },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_TLS_NELEM > 0
  { "pthread_getspecific", (FAR const void *)pthread_getspecific },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_gettid_np", (FAR const void *)pthread_gettid_np },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_join", (FAR const void *)pthread_join },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_TLS_NELEM > 0
  { "pthread_key_create", (FAR const void *)pthread_key_create },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_TLS_NELEM > 0
  { "pthread_key_delete", (FAR const void *)pthread_key_delete },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && !defined(CONFIG_PTHREAD_MUTEX_UNSAFE)
  { "pthread_mutex_consistent", (FAR const void *)pthread_mutex_consistent },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_destroy", (FAR const void *)pthread_mutex_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_PRIORITY_PROTECT)
  { "pthread_mutex_getprioceiling", (FAR const void *)pthread_mutex_getprioceiling },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_init", (FAR const void *)pthread_mutex_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_lock", (FAR const void *)pthread_mutex_lock },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_PRIORITY_PROTECT)
  { "pthread_mutex_setprioceiling", (FAR const void *)pthread_mutex_setprioceiling },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_timedlock", (FAR const void *)pthread_mutex_timedlock },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_trylock", (FAR const void *)pthread_mutex_trylock },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_unlock", (FAR const void *)pthread_mutex_unlock },
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
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_SMP)
  { "pthread_setaffinity_np", (FAR const void *)pthread_setaffinity_np },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_setname_np", (FAR const void *)pthread_setname_np },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_setschedparam", (FAR const void *)pthread_setschedparam },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_setschedprio", (FAR const void *)pthread_setschedprio },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_TLS_NELEM > 0
  { "pthread_setspecific", (FAR const void *)pthread_setspecific },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_sigmask", (FAR const void *)pthread_sigmask },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_yield", (FAR const void *)&pthread_yield },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "putenv", (FAR const void *)putenv },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "puts", (FAR const void *)puts },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "putwc_unlocked", (FAR const void *)putwc_unlocked },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "putwc", (FAR const void *)putwc },
#endif
  { "putwchar", (FAR const void *)putwchar },
  { "pwrite", (FAR const void *)pwrite },
  { "pwritev", (FAR const void *)pwritev },
  { "qsort", (FAR const void *)qsort },
  { "raise", (FAR const void *)raise },
  { "rand", (FAR const void *)&rand },
  { "read", (FAR const void *)read },
  { "readdir_r", (FAR const void *)readdir_r },
  { "readdir", (FAR const void *)readdir },
#if defined(CONFIG_PSEUDOFS_SOFTLINKS)
  { "readlink", (FAR const void *)readlink },
#endif
  { "readv", (FAR const void *)readv },
  { "realloc", (FAR const void *)realloc },
#if defined(CONFIG_NET)
  { "recv", (FAR const void *)recv },
#endif
#if defined(CONFIG_NET)
  { "recvfrom", (FAR const void *)recvfrom },
#endif
#if defined(CONFIG_NET)
  { "recvmsg", (FAR const void *)recvmsg },
#endif
  { "remove", (FAR const void *)remove },
  { "rename", (FAR const void *)rename },
#if defined(CONFIG_FILE_STREAM)
  { "rewind", (FAR const void *)rewind },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "rint", (FAR const void *)rint },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "rintf", (FAR const void *)rintf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "rintl", (FAR const void *)rintl },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "rmdir", (FAR const void *)rmdir },
#endif
#if defined(CONFIG_MODULE)
  { "rmmod", (FAR const void *)rmmod },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "round", (FAR const void *)round },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "roundf", (FAR const void *)roundf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "roundl", (FAR const void *)roundl },
#endif
  { "scandir", (FAR const void *)scandir },
#if defined(CONFIG_SCHED_BACKTRACE)
  { "sched_backtrace", (FAR const void *)sched_backtrace },
#endif
  { "sched_get_priority_max", (FAR const void *)sched_get_priority_max },
  { "sched_get_priority_min", (FAR const void *)sched_get_priority_min },
#if defined(CONFIG_SMP)
  { "sched_getaffinity", (FAR const void *)sched_getaffinity },
#endif
  { "sched_getcpu", (FAR const void *)&sched_getcpu },
  { "sched_getparam", (FAR const void *)sched_getparam },
  { "sched_getscheduler", (FAR const void *)sched_getscheduler },
  { "sched_lock", (FAR const void *)&sched_lock },
  { "sched_lockcount", (FAR const void *)&sched_lockcount },
  { "sched_rr_get_interval", (FAR const void *)sched_rr_get_interval },
#if defined(CONFIG_SMP)
  { "sched_setaffinity", (FAR const void *)sched_setaffinity },
#endif
  { "sched_setparam", (FAR const void *)sched_setparam },
  { "sched_setscheduler", (FAR const void *)sched_setscheduler },
  { "sched_unlock", (FAR const void *)&sched_unlock },
  { "sched_yield", (FAR const void *)&sched_yield },
  { "select", (FAR const void *)select },
  { "sem_getvalue", (FAR const void *)sem_getvalue },
  { "sem_init", (FAR const void *)sem_init },
#if defined(CONFIG_NET)
  { "send", (FAR const void *)send },
#endif
  { "sendfile", (FAR const void *)sendfile },
#if defined(CONFIG_NET)
  { "sendmsg", (FAR const void *)sendmsg },
#endif
#if defined(CONFIG_NET)
  { "sendto", (FAR const void *)sendto },
#endif
#if defined(CONFIG_SCHED_USER_IDENTITY)
  { "setegid", (FAR const void *)setegid },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "setenv", (FAR const void *)setenv },
#endif
#if defined(CONFIG_SCHED_USER_IDENTITY)
  { "seteuid", (FAR const void *)seteuid },
#endif
#if defined(CONFIG_SCHED_USER_IDENTITY)
  { "setgid", (FAR const void *)setgid },
#endif
  { "sethostname", (FAR const void *)sethostname },
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "setitimer", (FAR const void *)setitimer },
#endif
#if defined(CONFIG_LIBC_LOCALE)
  { "setlocale", (FAR const void *)setlocale },
#endif
  { "setlogmask", (FAR const void *)setlogmask },
  { "setpriority", (FAR const void *)setpriority },
#if defined(CONFIG_NET) && defined(CONFIG_NET_SOCKOPTS)
  { "setsockopt", (FAR const void *)setsockopt },
#endif
  { "settimeofday", (FAR const void *)settimeofday },
#if defined(CONFIG_SCHED_USER_IDENTITY)
  { "setuid", (FAR const void *)setuid },
#endif
#if defined(CONFIG_FS_SHMFS)
  { "shm_open", (FAR const void *)shm_open },
#endif
#if defined(CONFIG_FS_SHMFS)
  { "shm_unlink", (FAR const void *)shm_unlink },
#endif
#if defined(CONFIG_MM_SHM)
  { "shmat", (FAR const void *)shmat },
#endif
#if defined(CONFIG_MM_SHM)
  { "shmctl", (FAR const void *)shmctl },
#endif
#if defined(CONFIG_MM_SHM)
  { "shmdt", (FAR const void *)shmdt },
#endif
#if defined(CONFIG_MM_SHM)
  { "shmget", (FAR const void *)shmget },
#endif
#if defined(CONFIG_NET)
  { "shutdown", (FAR const void *)shutdown },
#endif
  { "sigaction", (FAR const void *)sigaction },
  { "sigaddset", (FAR const void *)sigaddset },
  { "sigdelset", (FAR const void *)sigdelset },
  { "sigemptyset", (FAR const void *)sigemptyset },
  { "sigfillset", (FAR const void *)sigfillset },
  { "sigismember", (FAR const void *)sigismember },
  { "signal", (FAR const void *)signal },
#if defined(CONFIG_SIGNAL_FD)
  { "signalfd", (FAR const void *)signalfd },
#endif
  { "sigpending", (FAR const void *)sigpending },
  { "sigprocmask", (FAR const void *)sigprocmask },
  { "sigqueue", (FAR const void *)sigqueue },
  { "sigsuspend", (FAR const void *)sigsuspend },
  { "sigtimedwait", (FAR const void *)sigtimedwait },
  { "sigwaitinfo", (FAR const void *)sigwaitinfo },
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "sin", (FAR const void *)sin },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "sinf", (FAR const void *)sinf },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "sinh", (FAR const void *)sinh },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "sinhf", (FAR const void *)sinhf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "sinhl", (FAR const void *)sinhl },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "sinl", (FAR const void *)sinl },
#endif
  { "sleep", (FAR const void *)sleep },
  { "snprintf", (FAR const void *)snprintf },
#if defined(CONFIG_NET)
  { "socket", (FAR const void *)socket },
#endif
#if defined(CONFIG_NET)
  { "socketpair", (FAR const void *)socketpair },
#endif
  { "sprintf", (FAR const void *)sprintf },
  { "sq_addafter", (FAR const void *)sq_addafter },
  { "sq_remafter", (FAR const void *)sq_remafter },
  { "sq_remfirst", (FAR const void *)sq_remfirst },
  { "sq_remlast", (FAR const void *)sq_remlast },
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "sqrt", (FAR const void *)sqrt },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "sqrtf", (FAR const void *)sqrtf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "sqrtl", (FAR const void *)sqrtl },
#endif
  { "srand", (FAR const void *)srand },
  { "sscanf", (FAR const void *)sscanf },
  { "sscanf", (FAR const void *)sscanf },
  { "stat", (FAR const void *)stat },
  { "statfs", (FAR const void *)statfs },
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
  { "strerror_r", (FAR const void *)strerror_r },
  { "strerror", (FAR const void *)strerror },
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
  { "strtok_r", (FAR const void *)strtok_r },
  { "strtok", (FAR const void *)strtok },
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
#if defined(CONFIG_PSEUDOFS_SOFTLINKS)
  { "symlink", (FAR const void *)symlink },
#endif
  { "sync", (FAR const void *)&sync },
  { "sysconf", (FAR const void *)sysconf },
  { "sysinfo", (FAR const void *)sysinfo },
  { "syslog", (FAR const void *)syslog },
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "tan", (FAR const void *)tan },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "tanf", (FAR const void *)tanf },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "tanh", (FAR const void *)tanh },
#endif
#if !defined(CONFIG_LIBM_NONE)
  { "tanhf", (FAR const void *)tanhf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "tanhl", (FAR const void *)tanhl },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && !defined(CONFIG_LIBM_NONE)
  { "tanl", (FAR const void *)tanl },
#endif
#if !defined(CONFIG_BUILD_KERNEL)
  { "task_create", (FAR const void *)task_create },
#endif
#if !defined(CONFIG_BUILD_KERNEL)
  { "task_delete", (FAR const void *)task_delete },
#endif
#if !defined(CONFIG_BUILD_KERNEL)
  { "task_restart", (FAR const void *)task_restart },
#endif
#if !defined(CONFIG_BUILD_KERNEL)
  { "task_spawn", (FAR const void *)task_spawn },
#endif
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
  { "tgkill", (FAR const void *)tgkill },
  { "time", (FAR const void *)time },
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "timer_create", (FAR const void *)timer_create },
#endif
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "timer_delete", (FAR const void *)timer_delete },
#endif
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "timer_getoverrun", (FAR const void *)timer_getoverrun },
#endif
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "timer_gettime", (FAR const void *)timer_gettime },
#endif
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "timer_settime", (FAR const void *)timer_settime },
#endif
#if defined(CONFIG_TIMER_FD)
  { "timerfd_create", (FAR const void *)timerfd_create },
#endif
#if defined(CONFIG_TIMER_FD)
  { "timerfd_gettime", (FAR const void *)timerfd_gettime },
#endif
#if defined(CONFIG_TIMER_FD)
  { "timerfd_settime", (FAR const void *)timerfd_settime },
#endif
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
#if !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "umount2", (FAR const void *)umount2 },
#endif
  { "uname", (FAR const void *)uname },
#if defined(CONFIG_FILE_STREAM)
  { "ungetc", (FAR const void *)ungetc },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "ungetwc_unlocked", (FAR const void *)ungetwc_unlocked },
#endif
#if defined(CONFIG_FILE_STREAM)
  { "ungetwc", (FAR const void *)ungetwc },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "unlink", (FAR const void *)unlink },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "unsetenv", (FAR const void *)unsetenv },
#endif
#if defined(CONFIG_ARCH_HAVE_FORK)
  { "up_fork", (FAR const void *)&up_fork },
#endif
  { "usleep", (FAR const void *)usleep },
  { "utimens", (FAR const void *)utimens },
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
#if defined(CONFIG_SCHED_WAITPID) && defined(CONFIG_SCHED_HAVE_PARENT)
  { "wait", (FAR const void *)wait },
#endif
#if defined(CONFIG_SCHED_WAITPID) && defined(CONFIG_SCHED_HAVE_PARENT)
  { "waitid", (FAR const void *)waitid },
#endif
#if defined(CONFIG_SCHED_WAITPID)
  { "waitpid", (FAR const void *)waitpid },
#endif
  { "wcrtomb", (FAR const void *)wcrtomb },
  { "wcscat", (FAR const void *)wcscat },
  { "wcschr", (FAR const void *)wcschr },
  { "wcscmp", (FAR const void *)wcscmp },
  { "wcscoll", (FAR const void *)wcscoll },
  { "wcscpy", (FAR const void *)wcscpy },
  { "wcscspn", (FAR const void *)wcscspn },
  { "wcsftime", (FAR const void *)wcsftime },
  { "wcslcat", (FAR const void *)wcslcat },
  { "wcslcpy", (FAR const void *)wcslcpy },
  { "wcslen", (FAR const void *)wcslen },
  { "wcsncat", (FAR const void *)wcsncat },
  { "wcsncmp", (FAR const void *)wcsncmp },
  { "wcsncpy", (FAR const void *)wcsncpy },
  { "wcsnrtombs", (FAR const void *)wcsnrtombs },
  { "wcspbrk", (FAR const void *)wcspbrk },
  { "wcsrchr", (FAR const void *)wcsrchr },
  { "wcsrtombs", (FAR const void *)wcsrtombs },
  { "wcsspn", (FAR const void *)wcsspn },
  { "wcsstr", (FAR const void *)wcsstr },
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
  { "write", (FAR const void *)write },
  { "writev", (FAR const void *)writev },

  /* ARM EABI / libgcc helpers */

  { "__aeabi_f2d", (FAR const void *)__aeabi_f2d },
  { "__aeabi_uldivmod", (FAR const void *)__aeabi_uldivmod },
  { "__aeabi_unwind_cpp_pr0", (FAR const void *)__aeabi_unwind_cpp_pr0 },
  { "__aeabi_unwind_cpp_pr1", (FAR const void *)__aeabi_unwind_cpp_pr1 },

  /* NuttX internal APIs */

  { "clock_systime_ticks", (FAR const void *)clock_systime_ticks }
};

#define NSYMBOLS (sizeof(g_symtab) / sizeof (struct symtab_s))
int g_nsymbols = NSYMBOLS;
