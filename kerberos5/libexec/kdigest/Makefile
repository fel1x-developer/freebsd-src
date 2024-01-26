
PACKAGE=	kerberos

PROG=	kdigest
MAN=	kdigest.8
CFLAGS+=	-I${KRB5DIR}/lib/asn1 \
		-I${KRB5DIR}/lib/roken \
		-I${KRB5DIR}/lib/sl -I.
LIBADD=	asn1 crypt edit hcrypto heimntlm krb5 roken sl vers
SRCS=	kdigest.c \
	kdigest-commands.c \
	kdigest-commands.h

CLEANFILES=	kdigest-commands.h kdigest-commands.c

kdigest-commands.h: kdigest-commands.in
	${SLC} ${.ALLSRC:M*.in}

kdigest-commands.c: kdigest-commands.h

.include <bsd.prog.mk>

.PATH: ${KRB5DIR}/kuser
