# Chain-CUDD
Extension of CUDD to support BDD with chain nodes.

Chain nodes are like regular BDD nodes, except they contain a second
index "bindex", which is >= the normal index.  A chain node N with
index t (for "top"), bindex b (for "bottom") and children T and E is
like a chain of BDD nodes with indices t, t+1, ..., b, linked through
their Else branches and all having Then branches equal to E.  The
bottommost node has T as its Then child.

ZDD nodes are a special case of chain nodes, have E be a constant
(nominally Zero, but with complement arcs they may be One).

The extension is implemented by including two index fields within the
regular BDD node.  When compiled for 64 bits, each index is 16 bits
long, which is enough for most applications.  When compiled for 32
bits, each index is 8 bits long, which really is not a good idea.

The code here was created by editting the code for CUDD v2.5.1.  Chain
is enabled with compile-time constant USE_CHAINING.  This value can
have 3 levels:

0: Don't use chaining.  Reverts to standard BDD functionality
(although underlying code has changed)

1: Use chaining.  This is the default

2: Use chaining, but only when the Else child is a constant.  This
   puts it closer to ZBDDs.  Mainly of experimental interest.

Changes required:

cudd.h: Revised node declaration

cuddInter.h, cuddTable.c:

Created separate function cuddUniqueInterChained.  This one has a
separate argument for the bindex.  The exisiting cuddUniqueInter is
kept to simplify code migration.  It handles the case where bindex =
index.  

cuddInter.h
Created separate macro ddHash2 to hash two dds + two ints



