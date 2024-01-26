
PACKAGE=	kerberos

PROG=	kcm
MAN=	kcm.8

SRCS=	acl.c \
	acquire.c \
	cache.c \
	client.c \
	config.c \
	connect.c \
	events.c \
	glue.c \
	headers.h \
	kcm_locl.h \
	log.c \
	main.c \
	protocol.c \
	sessions.c \
	renew.c

CFLAGS+=-I${KRB5DIR}/lib/krb5 -I${KRB5DIR}/lib/asn1 -I${KRB5DIR}/lib/roken \
	-I${KRB5DIR}/kcm -I${KRB5DIR}/lib/ipc ${LDAPCFLAGS}
# Avoid errors for using deprecated krb5_* APIs
CFLAGS+="-DKRB5_DEPRECATED_FUNCTION(X)="
LIBADD=	hdb krb5 asn1 roken heimntlm heimipcs hcrypto
DPADD=	${LDAPDPADD}
LDADD=	${LIBVERS} ${LDAPLDADD}
LDFLAGS=${LDAPLDFLAGS}

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/kcm
