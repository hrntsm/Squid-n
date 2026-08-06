// 全ページの本文末尾に、ライセンスと無保証を明示するフッターを差し込む。
// mdBook のテンプレート（index.hbs）を丸ごと上書きすると mdBook の更新に追従できなくなるため、
// 追加スクリプトから DOM を組み立てる方式にしている。
(function () {
    "use strict";

    var main = document.querySelector("main");
    if (!main) {
        return;
    }

    // mdBook が index.hbs で定義するグローバル変数。ページ階層に応じた相対パスが入る。
    var root = typeof path_to_root === "string" ? path_to_root : "";

    var footer = document.createElement("footer");
    footer.className = "site-footer";

    var copyright = document.createElement("p");
    copyright.className = "site-footer-copyright";
    copyright.textContent = "© 2026 Hiroaki NATSUME";

    var notice = document.createElement("p");
    notice.className = "site-footer-notice";

    var licenseLink = document.createElement("a");
    licenseLink.href = "https://github.com/hrntsm/squid-n/blob/main/LICENSE";
    licenseLink.textContent = "MIT License";

    var disclaimerLink = document.createElement("a");
    disclaimerLink.href = root + "introduction.html#ライセンスと免責事項";
    disclaimerLink.textContent = "ライセンスと免責事項";

    notice.appendChild(document.createTextNode("Squid-n は "));
    notice.appendChild(licenseLink);
    notice.appendChild(
        document.createTextNode(
            " のもとで「現状のまま（AS IS）」提供するソフトウェアです。計算結果および本ドキュメントについて、いかなる保証も行いません（",
        ),
    );
    notice.appendChild(disclaimerLink);
    notice.appendChild(document.createTextNode("）。"));

    footer.appendChild(copyright);
    footer.appendChild(notice);
    main.appendChild(footer);
})();
