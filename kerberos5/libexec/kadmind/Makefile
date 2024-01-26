
PACKAGE=	kerberos

PROG=	kadmind
MAN=	kadmind.8

SRCS=	rpc.c \
	server.c \
	kadmind.c \
	kadm_conn.c

SRCS=	gss_oid.c

CSTD=	gnu11
CFLAGS+=-I${KRB5DIR}/lib/krb5 -I${KRB5DIR}/lib/asn1 \
	-I${SRCTOP}/kerberos5/include/gssapi ${LDAPCFLAGS}
LIBADD=	kadm5srv kadm5clnt hcrypto asn1 gssapi hdb krb5 roken vers
DPADD=	${LDAPDPADD}
LDADD=	${LDAPLDADD}
LDFLAGS=${LDAPLDFLAGS}

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/kadmin ${KRB5DIR}/lib/gssapi/mech
