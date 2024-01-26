
PACKAGE=	kerberos

PROG=	heimtools
MAN=
CFLAGS+=	-I${KRB5DIR}/lib/asn1 \
		-I${KRB5DIR}/lib/roken \
		-I.
LIBADD=	krb5 roken asn1 gssapi heimntlm hcrypto sl edit heimbase hx509 vers kafs5

SRCS=	heimtools.c \
	klist.c \
	kswitch.c \
	copy_cred_cache.c \
	heimtools-commands.c \
	heimtools-commands.h

CLEANFILES=	heimtools-commands.h heimtools-commands.c

.include <bsd.prog.mk>

heimtools-commands.h: ${KRB5DIR}/kuser/heimtools-commands.in
	${SLC} ${.ALLSRC:M*.in}

heimtools-commands.c: heimtools-commands.h

.PATH: ${KRB5DIR}/kuser
