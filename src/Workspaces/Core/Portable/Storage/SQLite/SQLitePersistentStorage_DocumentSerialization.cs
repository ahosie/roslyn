﻿// Copyright (c) Microsoft.  All Rights Reserved.  Licensed under the Apache License, Version 2.0.  See License.txt in the project root for license information.

using System.IO;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.CodeAnalysis.SQLite.Interop;

namespace Microsoft.CodeAnalysis.SQLite
{
    internal partial class SQLitePersistentStorage
    {
        public override Task<Checksum> ReadChecksumAsync(Document document, string name, CancellationToken cancellationToken)
            => Task.FromResult(_documentAccessor.ReadChecksum((document, name), cancellationToken));

        public override Task<Stream> ReadStreamAsync(Document document, string name, Checksum checksum, CancellationToken cancellationToken = default)
            => Task.FromResult(_documentAccessor.ReadStream((document, name), checksum, cancellationToken));

        public override Task<bool> WriteStreamAsync(Document document, string name, Stream stream, Checksum checksum, CancellationToken cancellationToken = default)
            => Task.FromResult(_documentAccessor.WriteStream((document, name), stream, checksum, cancellationToken));

        /// <summary>
        /// <see cref="Accessor{TKey, TWriteQueueKey, TDatabaseId}"/> responsible for storing and 
        /// retrieving data from <see cref="DocumentDataTableName"/>.
        /// </summary>
        private class DocumentAccessor : Accessor<
            (Document document, string name),
            (DocumentId, string),
            long>
        {
            public DocumentAccessor(SQLitePersistentStorage storage) : base(storage)
            {
            }

            protected override string DataTableName => DocumentDataTableName;

            protected override (DocumentId, string) GetWriteQueueKey((Document document, string name) key)
                => (key.document.Id, key.name);

            protected override bool TryGetDatabaseId(SqlConnection connection, (Document document, string name) key, out long dataId)
                => Storage.TryGetDocumentDataId(connection, key.document, key.name, out dataId);

            protected override void BindFirstParameter(SqlStatement statement, long dataId)
                => statement.BindInt64Parameter(parameterIndex: 1, value: dataId);

            protected override bool TryGetRowIdFromMainDB(SqlConnection connection, long dataId, out long rowId)
                => GetAndVerifyRowId(connection, dataId, out rowId);
        }
    }
}
