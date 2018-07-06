AXISバックエンドの実装方法について
文責：東京大学 千代 浩之

[はじめに]
- 利用したLLVMのバージョンは6.0.0
- MIPSのバックエンドをベースに実装


[名前をAXISからMAXISに変更した理由]
名前をAXISにすると自作プログラムでコンパイルした際，sub命令がないというエラーが発生した．
MAXISにすると問題が解決したので，最初の文字が重要と思われる．
MIPS依存コードを全て除去すればAXISという名前でも大丈夫かもしれない．
AXISをMAXISとして説明する．


[バックエンド&フロントエンド共通]
http://releases.llvm.org/からLLVM/Clang/LLDをダウンロードする．
ClangとLLDはllvm/tools/以下に配置．


基本的には，Mipsと書いてある箇所をMaxisに追加修正する方針
$ grep mips * -Ri
で(大文字・小文字の区別なしの)mipsの文字列を検索する

[置換ルール]
名前がかぶっている関数がある場合はMaxisという文字列を関数に入れる．
[プログラム中でのif文での分岐方法]
if (Config->EMachine == EM_MAXIS)という形に書く

ファイル名にMipsと書いているものはコピーしてMaxisに変更する
Mips.cpp -> Maxis.cpp


[バックエンドの実装方法]

過去のLLVMのコミットログが参考になる．
https://github.com/pflab-ut/llvm/commits/master


- 命令を追加する方法
1. ADD/SUB命令等はMaxisInstrInfo.tdファイルを編集する
2. XNOR/NAND命令は1の方法と一緒に以下のファイルでXOR命令をコピーと修正
include/llvm/CodeGen/GlobalISel/IRTranslator.h
include/llvm/CodeGen/ISDOpcodes.h
include/llvm/CodeGen/TargetOpcodes.def
include/llvm/Target/GenericOpcodes.td
include/llvm/Target/GlobalISel/SelectionDAGCompat.td
include/llvm/Target/TargetSelectionDAG.td
lib/Target/Maxis/MaxisInstrInfo.td
lib/Target/Maxis/MaxisSchedule.td
※LLVMのバージョンによってXOR命令の実装ファイルが変わる可能性が
あるのでバージョン毎に「$ grep XOR * -Ri」を実行する．


- DelaySlotを1から0にする方法
lib/Target/Maxis/MaxisInstrInfo.tdファイルでhasDelaySlot = 1をhasDelaySlot = 0にする


- llvm-objdumpで出力する命令をalias名に変更する方法
lib/Target/Maxis/InstPrinter/MaxisInstPrinter.cppの
printAliasメンバ関数を変更する


- x86にxnor命令があるので，以下のファイルでxnorを_xnorにしている．
include/llvm/Target/GlobalISel/SelectionDAGCompat.td
include/llvm/Target/TargetSelectionDAG.td
lib/Target/Maxis/MaxisInstrInfo.td


[フロントエンド]

変数のalignを変更する場所は，
tools/clang/lib/Basic/Targets/Maxis.h

ただし，tools/clang/include/clang/Basic/TargetInfo.hの
getCharAlign/getShortAlign関数をvirtualにしないと変更できない．

