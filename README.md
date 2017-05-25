# Chain-CUDD
Extension of CUDD to support BDD with chain nodes.

Chain nodes are like regular BDD nodes, except they contain a second
index "bindex", which is >= the normal index.  A chain node N with
index t (for "top"), bindex b (for "bottom") and children T and E is
like a chain of BDD nodes with indices t, t+1, ..., b, linked through
their Else branches and all having Then branches equal to T.  The
bottommost node has T as its Else child.

ZDD nodes are a special case of chain nodes, have E be a constant
(nominally Zero, but with complement arcs they may be One).

The extension is implemented by including two index fields within the
regular BDD node.  When compiled for 64 bits, each index is 16 bits
long, which is enough for most applications.  When compiled for 32
bits, each index is 8 bits long, which really is not a good idea.

The code here was created by editting the code for CUDD v2.5.1.
The chaining mode is set for a Dd manager using the function
Cudd_SetChaining.  There are 3 levels, defined by an enumerated type:

CUDD_CHAIN_NONE: Don't use chaining.  Reverts to standard BDD
functionality (although underlying code has changed)

CUDD_CHAIN_CONSTANT: Apply chain reduction only when Then child is
constant.  This puts it closer to ZDDs.  Mainly of experimental
interest.

CUDD_CHAIN_ALL: Apply chain reduction whenever possible.

Changes required. Comment "Chaining Support" appears every place in code


cudd.h: Revised node declaration

cuddInt.h, cuddTable.c:

Created separate function cuddUniqueInterChained.  This one has a
separate argument for the bindex.  The exisiting cuddUniqueInter is
kept to simplify code migration.  It handles the case where bindex =
index.

cuddInt.h:
Created separate macro ddHash2 to hash two dds + two ints

cuddBddIte.c:

Implement revised versions of ITE, And, and XOR operations.
Refactored code to improve its modularity.  Different levels of chain
compression implemented by function bddGenerateNode.


