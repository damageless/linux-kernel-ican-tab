Index: drivers/mtd/nand/imapx200.c
===================================================================
--- drivers/mtd/nand/imapx200.c	(revision 2911)
+++ drivers/mtd/nand/imapx200.c	(working copy)
@@ -104,9 +104,13 @@
 		.offset     = 296 * SZ_1M,
 		.size       = 192 * SZ_1M,
 	}, {
+		.name       = "xxx",
+		.offset     = MTDPART_OFS_APPEND,
+		.size       = 256 * SZ_1M,
+	}, {
 		.name       = "userdata",
 		.offset     = MTDPART_OFS_APPEND,
-		.size       = 984 * SZ_1M,
+		.size       = 728 * SZ_1M,
 	}, {
 		.name       = "cache",
 		.offset     = MTDPART_OFS_APPEND,
