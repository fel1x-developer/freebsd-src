
PACKAGE=	kerberos

PROG=	ipropd-master
MAN=	iprop.8
SRCS=	ipropd_common.c ipropd_master.c kadm5_err.h
CFLAGS+=-I${KRB5DIR}/lib/krb5 -I${KRB5DIR}/lib/asn1 -I${KRB5DIR}/lib/roken \
	-I. ${LDAPCFLAGS}
LIBADD=	crypt hdb kadm5srv krb5 roken vers
DPADD=	${LDAPDPADD}
LDADD=	${LDAPLDADD}
LDFLAGS=${LDAPLDFLAGS}

WARNS?=	0

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/lib/kadm5
