
PACKAGE=	kerberos

PROG=	kpasswdd
MAN=	kpasswdd.8
SRCS=	kpasswdd.c \
	kpasswd_locl.h
CFLAGS+=-I${KRB5DIR}/lib/roken -I${KRB5DIR}/lib/libhdb ${LDAPCFLAGS}
LIBADD=	asn1 crypt hcrypto hdb kadm5srv krb5 roken vers
DPADD=	${LDAPDPADD}
LDADD=	${LDAPLDADD}
LDFLAGS=${LDAPLDFLAGS}

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/kpasswd
