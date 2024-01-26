
PACKAGE=	kerberos

PROG=	kpasswd
MAN=	kpasswd.1
CFLAGS+=-I${KRB5DIR}/lib/roken
SRCS=	kpasswd.c \
	kpasswd_locl.h
LIBADD=	krb5 roken asn1 hcrypto vers
LDFLAGS=${LDAPLDFLAGS}

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/kpasswd
