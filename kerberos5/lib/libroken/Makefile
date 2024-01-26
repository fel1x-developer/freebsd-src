
PACKAGE=	kerberos-lib

LIB=	roken
MAN=	getarg.3 parse_time.3 rtbl.3 ecalloc.3
VERSION_MAP=	${KRB5DIR}/lib/roken/version-script.map
LIBADD=	crypt

INCS=	roken.h \
	roken-common.h \
	base64.h \
	getarg.h \
	hex.h \
	parse_bytes.h \
	parse_time.h \
	parse_units.h \
	resolve.h \
	rtbl.h \
	xdbm.h

SRCS=	base64.c \
	bswap.c \
	cloexec.c \
	concat.c \
	copyhostent.c \
	ct.c \
	daemon.c \
	detach.c \
	doxygen.c \
	dumpdata.c \
	ecalloc.c \
	emalloc.c \
	environment.c \
	eread.c \
	erealloc.c \
	esetenv.c \
	estrdup.c \
	ewrite.c \
	get_default_username.c \
	get_window_size.c \
	getaddrinfo_hostspec.c \
	getarg.c \
	getcap.c \
	getnameinfo_verified.c \
	getprogname.c \
	getxxyyy.c \
	h_errno.c \
	hex.c \
	hostent_find_fqdn.c \
	issuid.c \
	k_getpwnam.c \
	k_getpwuid.c \
	mini_inetd.c \
	mkdir.c \
	mkstemp.c \
	ndbm_wrap.c \
	net_read.c \
	net_write.c \
	parse_bytes.c \
	parse_time.c \
	parse_units.c \
	qsort.c \
	rand.c \
	realloc.c \
	resolve.c \
	roken_gethostby.c \
	rtbl.c \
	setprogname.c \
	signal.c \
	simple_exec.c \
	snprintf.c \
	socket.c \
	strcollect.c \
	strerror_r.c \
	strlwr.c \
	strpool.c \
	strsep_copy.c \
	strupr.c \
	timeval.c \
	tm2time.c \
	unvis.c \
	verify.c \
	vis.c \
	warnerr.c \
	write_pid.c \
	xfree.c

CFLAGS+=-I${KRB5DIR}/lib/roken \
	-I${SRCTOP}/kerberos5/include \
	-I${KRB5DIR}/lib/krb5 -I.

CLEANFILES= roken.h

roken.h:
	${MAKE_ROKEN} > ${.TARGET}

.include <bsd.lib.mk>

.PATH: ${KRB5DIR}/lib/roken
