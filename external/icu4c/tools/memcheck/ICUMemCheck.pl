$fileNames = `find common i18n layout io -name "*.o" -print`;
foreach $f (split('\n', $fileNames)) {
   $symbols = `nm -u -C $f`;
   if ($symbols =~ /U +operator delete\(void\*\)/) {
      print "$f 	$&\n";
   }
   if ($symbols =~ /U +operator delete\[\]\(void\*\)/) {
      print "$f 	$&\n";
   }
   if ($symbols =~ /U +operator new\(unsigned int\)/) {
      print "$f 	$&\n";
   }
   if ($symbols =~ /U +operator new\[\]\(unsigned int\)/) {
      print "$f 	$&\n";
   }
   if ($symbols =~ /U +malloc.*/) {
      print "$f 	$&\n";
   }
   if ($symbols =~ /U +free.*/) {
      print "$f 	$&\n";
   }

}
