from lab6.tree import *

if __name__ == "__main__":
    arr = []
    bang = TreeNode("!")
    t = TreeNode("hello", {
        TreeNode("world"),
        bang
    })
    t.pprint()
    bang.make_root()

    bang.pprint()
    t.pprint()

    tree = ArrayBackedTree(EdgeTreeNode, arr)
    ETreeNode = tree.Node

    print ETreeNode.__mro__

    c = ETreeNode("C", 'A->C')
    b = ETreeNode("B", "A->B", {
        ETreeNode("D", "B->D"),
        ETreeNode("D", 'B->D')
    })

    t = ETreeNode("A", children={
        b,
        c,
        ETreeNode("D2", 'A->D2')
    })

    print "Initial"
    t.pprint()

    print "Full tree"
    for it in t.iter:
        print(it.value)

    print "skipped tree"
    it = t.iter
    for n in it:
        print(n.value)
        if n.value == 'D2':
            it.skip_children()

    c.make_root()

    print "Rerooted"
    c.pprint()
    print "Initial subtree"
    t.pprint()

    b.parent = None
    print "Removed"
    c.pprint()

    del b
    print [i()==None for i in tree._node_arr]
    tree.pack()
    print [i()==None for i in tree._node_arr]

    c.pprint()
